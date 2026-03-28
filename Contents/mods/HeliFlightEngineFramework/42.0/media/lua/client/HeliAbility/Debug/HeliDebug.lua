--[[
    HeliDebug — Debug logging, state capture, and flight data recorder

    Provides:
    - Toggle-able periodic console logging (1 line per LOG_INTERVAL ticks)
    - One-shot snap capture (triggered by /hef snap)
    - Last-frame state storage (read by /hef snap in HeliDebugCommands)
    - Flight data recorder: writes per-frame CSV to Zomboid directory for offline analysis

    Controlled by: /hef log, /hef snap, /hef record (in HeliDebugCommands.lua)
    Written by: HeliMove.lua orchestrator
]]

HeliDebug = {}

HeliDebug.logEnabled = false       -- throttled periodic logging (to Lua console)
HeliDebug.snapRequested = false    -- one-shot detailed dump (read by HeliDebugCommands)
HeliDebug.lastLogTick = 0
HeliDebug.LOG_INTERVAL = 12        -- ticks between log lines (~5x/sec at 60 FPS)

-- Last-frame state (written by orchestrator, read by /hef snap command)
HeliDebug.lastState = {}

-------------------------------------------------------------------------------------
-- Capture flight state for debug display
-------------------------------------------------------------------------------------
function HeliDebug.captureState(state)
    HeliDebug.lastState = state
end

-------------------------------------------------------------------------------------
-- Throttled periodic log line to Lua console
-------------------------------------------------------------------------------------
function HeliDebug.periodicLog(state, currentAltitude, desiredHX, desiredHZ, simVX, simVZ, errX, errZ, savedVelY, keyStr)
    if not HeliDebug.logEnabled then return end
    HeliDebug.lastLogTick = HeliDebug.lastLogTick + 1
    if HeliDebug.lastLogTick >= HeliDebug.LOG_INTERVAL then
        HeliDebug.lastLogTick = 0
        print(string.format(
            "[HEF] %s z=%.2f des=(%.1f,%.1f) simV=(%.1f,%.1f) err=(%.2f,%.2f) svy=%.1f keys=%s",
            state, currentAltitude,
            desiredHX, desiredHZ,
            simVX, simVZ,
            errX, errZ,
            savedVelY,
            keyStr))
    end
end

-------------------------------------------------------------------------------------
-- Engine-off log line
-------------------------------------------------------------------------------------
function HeliDebug.logEngineOff(currentAltitude, fallSpeed, currentVelX, currentVelY, currentVelZ)
    if not HeliDebug.logEnabled then return end
    HeliDebug.lastLogTick = HeliDebug.lastLogTick + 1
    if HeliDebug.lastLogTick >= HeliDebug.LOG_INTERVAL then
        HeliDebug.lastLogTick = 0
        print(string.format("[HEF] ENG_OFF z=%.2f tgt=(0,%.1f,0) vel=(%.1f,%.1f,%.1f)",
            currentAltitude, -fallSpeed, currentVelX, currentVelY, currentVelZ))
    end
end

-------------------------------------------------------------------------------------
-- Flight data recorder — per-frame CSV for offline analysis
-------------------------------------------------------------------------------------

local _recorder = {
    active = false,
    writer = nil,
    frameCount = 0,
    filename = nil,
}

-- Base CSV columns (always present, framework-level)
local CSV_BASE_HEADER = "frame,ms,state,fps,actualX,actualZ,alt,velocityX,velocityY,velocityZ,desiredVelX,desiredVelZ,desiredVelY,subSteps,tilt,flightAssistOff,yawSimulated,yawActual,gravComp,dualPath,keys"

--- Start recording flight data to CSV.
--- Queries the active engine for additional columns via getDebugColumns().
--- File is written to Zomboid user directory (PZ sandbox restriction).
--- @return string|nil filename on success, nil on failure
function HeliDebug.startRecording()
    if _recorder.active then
        HeliDebug.stopRecording()
    end

    -- Capture engine columns at recording start (fixed for this session)
    local engineCols = HeliSimService.getDebugColumns()
    _recorder.engineColumns = engineCols or {}

    -- Build full header: base + engine-specific
    local header = CSV_BASE_HEADER
    for _, col in ipairs(_recorder.engineColumns) do
        header = header .. "," .. col
    end

    -- Timestamp-based filename using epoch millis (avoids Calendar API dependency)
    local filename = "HeliFlightLog_" .. tostring(getTimestampMs()) .. ".csv"

    local writer = getFileWriter(filename, true, false)  -- create new, don't append
    if not writer then
        print("[HEF] ERROR: Could not open " .. filename .. " for writing")
        return nil
    end

    writer:write(header .. "\n")
    _recorder.writer = writer
    _recorder.active = true
    _recorder.frameCount = 0
    _recorder.filename = filename
    print("[HEF] Recording started: " .. filename .. " (" .. #_recorder.engineColumns .. " engine columns)")
    return filename
end

--- Stop recording and close the file.
--- @return number frameCount Total frames recorded
function HeliDebug.stopRecording()
    if not _recorder.active then return 0 end
    local count = _recorder.frameCount
    local filename = _recorder.filename

    if _recorder.writer then
        _recorder.writer:close()
        _recorder.writer = nil
    end

    _recorder.active = false
    _recorder.frameCount = 0
    _recorder.filename = nil
    print("[HEF] Recording stopped: " .. tostring(filename) .. " (" .. count .. " frames)")
    return count
end

--- @return boolean
function HeliDebug.isRecording()
    return _recorder.active
end

--- @return string|nil Current recording filename
function HeliDebug.getRecordingFilename()
    return _recorder.filename
end

--- Write one frame of flight telemetry to the CSV.
--- Called from HeliMove orchestrator every airborne frame when recording is active.
--- Base columns are framework-level (always present). Engine columns are appended
--- dynamically from getDebugState(), keyed by column names captured at startRecording().
---
--- Base column reference:
---   frame, ms, state, fps, actualX, actualZ, alt, velocityX/Y/Z,
---   desiredVelX/Z/Y, subSteps, tilt, flightAssistOff, yawSimulated,
---   yawActual, gravComp, dualPath, keys
--- Engine columns: whatever getDebugColumns() returned at recording start.
function HeliDebug.writeFlightFrame(data, engineDebugState)
    if not _recorder.active or not _recorder.writer then return end

    _recorder.frameCount = _recorder.frameCount + 1

    -- Base columns (framework-level, always present)
    local line = string.format(
        "%d,%d,%s,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,%d,%d,%s",
        _recorder.frameCount,
        data.ms or 0,
        data.state or "?",
        data.fps or 0,
        data.actualX or 0,
        data.actualZ or 0,
        data.alt or 0,
        data.velocityX or 0,
        data.velocityY or 0,
        data.velocityZ or 0,
        data.desiredVelX or 0,
        data.desiredVelZ or 0,
        data.desiredVelY or 0,
        data.subSteps or 0,
        data.tilt and 1 or 0,
        data.flightAssistOff and 1 or 0,
        data.yawSimulated or 0,
        data.yawActual or 0,
        data.gravComp and 1 or 0,
        data.dualPath and 1 or 0,
        data.keys or "-")

    -- Engine-specific columns (from getDebugState, in getDebugColumns order)
    if engineDebugState and _recorder.engineColumns then
        for _, col in ipairs(_recorder.engineColumns) do
            local val = engineDebugState[col]
            if type(val) == "number" then
                line = line .. "," .. string.format("%.6f", val)
            elseif type(val) == "boolean" then
                line = line .. "," .. (val and "1" or "0")
            elseif val ~= nil then
                line = line .. "," .. tostring(val)
            else
                line = line .. ",0"
            end
        end
    end

    _recorder.writer:write(line .. "\n")
end
