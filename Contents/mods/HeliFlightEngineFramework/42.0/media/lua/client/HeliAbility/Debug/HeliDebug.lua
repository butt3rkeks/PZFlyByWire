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

-- CSV header matches writeFlightFrame column order exactly.
local CSV_HEADER = "frame,ms,state,fps,actualX,actualZ,alt,velocityX,velocityY,velocityZ,simPosX,simPosZ,simVelX,simVelZ,errorX,errorZ,desiredVelX,desiredVelZ,desiredVelY,subSteps,tilt,flightAssistOff,yawSimulated,yawActual,gravComp,dualPath,keys"

--- Start recording flight data to CSV.
--- File is written to Zomboid user directory (PZ sandbox restriction).
--- @return string|nil filename on success, nil on failure
function HeliDebug.startRecording()
    if _recorder.active then
        HeliDebug.stopRecording()
    end

    -- Timestamp-based filename using epoch millis (avoids Calendar API dependency)
    local filename = "HeliFlightLog_" .. tostring(getTimestampMs()) .. ".csv"

    local writer = getFileWriter(filename, true, false)  -- create new, don't append
    if not writer then
        print("[HEF] ERROR: Could not open " .. filename .. " for writing")
        return nil
    end

    writer:write(CSV_HEADER .. "\n")
    _recorder.writer = writer
    _recorder.active = true
    _recorder.frameCount = 0
    _recorder.filename = filename
    print("[HEF] Recording started: " .. filename)
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
---
--- Column reference:
---   frame     — monotonic frame counter (resets per recording session)
---   ms        — getTimestampMs() wall clock (milliseconds, for timing analysis)
---   state     — flight state string (warmup/airborne)
---   fps       — getAverageFPS() at this frame
---   actualX, actualZ    — actual position (PZ world X, Y)
---   alt                 — actual altitude (currentAltitude, Z-levels above ground datum)
---   velocityX,velocityY,velocityZ — Bullet velocity (X=world east, Y=height, Z=world south)
---   simPosX, simPosZ    — simulation model position (ideal trajectory)
---   simVelX, simVelZ    — simulation model velocity
---   errorX, errorZ      — position error (sim - actual, after tanh saturation)
---   desiredVelX, desiredVelZ — desired horizontal velocity (from tilt + heading)
---   desiredVelY         — desired vertical velocity (ascend/descend/hover target)
---   subSteps            — physics sub-steps this frame (integer, from accumulator)
---   tilt                — hasTiltInput flag (1=tilt active, 0=auto-leveling/stopped)
---   flightAssistOff     — flight assist off (1=free coast mode, 0=normal)
---   yawSimulated        — simulation yaw (degrees, _simYaw from HeliYawController)
---   yawActual           — actual yaw (degrees, vehicle:getAngleY())
---   gravComp            — gravity compensation active (1/0)
---   dualPath            — dual-path force system active (1/0)
---   keys                — key string (U/D/L/R/W/S/a/d or "-" for none)
function HeliDebug.writeFlightFrame(data)
    if not _recorder.active or not _recorder.writer then return end

    _recorder.frameCount = _recorder.frameCount + 1

    local line = string.format(
        "%d,%d,%s,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,%d,%d,%s",
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
        data.simPosX or 0,
        data.simPosZ or 0,
        data.simVelX or 0,
        data.simVelZ or 0,
        data.errorX or 0,
        data.errorZ or 0,
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

    _recorder.writer:write(line .. "\n")
end
