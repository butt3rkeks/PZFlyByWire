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
function HeliDebug.periodicLog(state, curr_z, desiredHX, desiredHZ, simVX, simVZ, errX, errZ, savedVelY, keyStr)
    if not HeliDebug.logEnabled then return end
    HeliDebug.lastLogTick = HeliDebug.lastLogTick + 1
    if HeliDebug.lastLogTick >= HeliDebug.LOG_INTERVAL then
        HeliDebug.lastLogTick = 0
        print(string.format(
            "[HEF] %s z=%.2f des=(%.1f,%.1f) simV=(%.1f,%.1f) err=(%.2f,%.2f) svy=%.1f keys=%s",
            state, curr_z,
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
function HeliDebug.logEngineOff(curr_z, fallSpeed, cvx, cvy, cvz)
    if not HeliDebug.logEnabled then return end
    HeliDebug.lastLogTick = HeliDebug.lastLogTick + 1
    if HeliDebug.lastLogTick >= HeliDebug.LOG_INTERVAL then
        HeliDebug.lastLogTick = 0
        print(string.format("[HEF] ENG_OFF z=%.2f tgt=(0,%.1f,0) vel=(%.1f,%.1f,%.1f)",
            curr_z, -fallSpeed, cvx, cvy, cvz))
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
local CSV_HEADER = "frame,ms,state,fps,aX,aZ,alt,vX,vY,vZ,sX,sZ,svX,svZ,eX,eZ,dvX,dvZ,dvY,sub,tilt,faOff,yawSim,yawAct,gravComp,dualPath,keys"

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
---   aX, aZ    — actual position (PZ world X, Y)
---   alt       — actual altitude (curr_z, Z-levels above ground datum)
---   vX,vY,vZ  — Bullet velocity (X=world east, Y=height, Z=world south)
---   sX, sZ    — simulation model position (ideal trajectory)
---   svX, svZ  — simulation model velocity
---   eX, eZ    — position error (sim - actual, after tanh saturation)
---   dvX, dvZ  — desired horizontal velocity (from tilt + heading)
---   dvY       — desired vertical velocity (ascend/descend/hover target)
---   sub       — physics sub-steps this frame (integer, from accumulator)
---   tilt      — hasTiltInput flag (1=tilt active, 0=auto-leveling/stopped)
---   faOff     — flight assist off (1=free coast mode, 0=normal)
---   yawSim    — simulation yaw (degrees, _simYaw from HeliYawController)
---   yawAct    — actual yaw (degrees, vehicle:getAngleY())
---   gravComp  — gravity compensation active (1/0)
---   dualPath  — dual-path force system active (1/0)
---   keys      — key string (U/D/L/R/W/S/a/d or "-" for none)
function HeliDebug.writeFlightFrame(data)
    if not _recorder.active or not _recorder.writer then return end

    _recorder.frameCount = _recorder.frameCount + 1

    local line = string.format(
        "%d,%d,%s,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,%d,%d,%s",
        _recorder.frameCount,
        data.ms or 0,
        data.state or "?",
        data.fps or 0,
        data.aX or 0,
        data.aZ or 0,
        data.alt or 0,
        data.vX or 0,
        data.vY or 0,
        data.vZ or 0,
        data.sX or 0,
        data.sZ or 0,
        data.svX or 0,
        data.svZ or 0,
        data.eX or 0,
        data.eZ or 0,
        data.dvX or 0,
        data.dvZ or 0,
        data.dvY or 0,
        data.sub or 0,
        data.tilt and 1 or 0,
        data.faOff and 1 or 0,
        data.yawSim or 0,
        data.yawAct or 0,
        data.gravComp and 1 or 0,
        data.dualPath and 1 or 0,
        data.keys or "-")

    _recorder.writer:write(line .. "\n")
end
