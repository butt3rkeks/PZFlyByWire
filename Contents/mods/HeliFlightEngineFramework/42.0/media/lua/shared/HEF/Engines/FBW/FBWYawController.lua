--[[
    FBWYawController — Yaw MPC (simYaw tracking + heading re-anchor)

    Tracks intended heading during A/D rotation. On key release, re-anchors
    simYaw from actual yaw. Detects large heading changes for sim position
    re-anchor (delegates snap decision to facade).

    Pure controller logic. Reads HeliConfig.TARGET_FPS, HeliConfig.MIN_FPS
    for threshold scaling.
]]

FBWYawController = {}

-------------------------------------------------------------------------------------
-- Yaw MPC state
-------------------------------------------------------------------------------------
local _simYaw = nil
local _wasRotating = false
local _lastSimYaw = nil

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset all yaw state.
function FBWYawController.reset()
    _simYaw = nil
    _wasRotating = false
    _lastSimYaw = nil
end

--- Update yaw MPC: track intended heading during rotation, re-anchor on release.
--- @param currentYawDeg number Current yaw from FBWOrientation (degrees)
--- @param isRotating boolean Whether A/D keys are pressed
--- @param yawDelta number Yaw change this frame (degrees, from FBWInputProcessor)
--- @return number simYaw The intended heading for hard lock
function FBWYawController.update(currentYawDeg, isRotating, yawDelta)
    if isRotating then
        if _simYaw then
            _simYaw = _simYaw + yawDelta
        else
            _simYaw = currentYawDeg
        end
        _wasRotating = true
    else
        if _wasRotating or not _simYaw then
            _simYaw = currentYawDeg
            _wasRotating = false
        end
        -- Hard lock: simYaw IS the authoritative heading when not rotating
    end

    return _simYaw
end

--- Check if heading has changed enough to warrant a sim position re-anchor.
--- Uses time-scaled threshold: 2 deg at TARGET_FPS, scaled proportionally.
--- @param fps number Current average FPS
--- @return boolean shouldReanchor, and internally updates _lastSimYaw
function FBWYawController.checkHeadingReanchor(fps)
    if not _simYaw or not _lastSimYaw then
        _lastSimYaw = _simYaw
        return false
    end

    local yawDelta = math.abs(_simYaw - _lastSimYaw)
    if yawDelta > 180 then yawDelta = 360 - yawDelta end

    local reanchorThreshold = 2.0 * HeliConfig.TARGET_FPS / math.max(fps, HeliConfig.MIN_FPS)
    _lastSimYaw = _simYaw

    if yawDelta > reanchorThreshold then
        return true
    end

    return false
end

--- Get current simYaw (intended heading).
--- @return number|nil Yaw in degrees, or nil if not initialized
function FBWYawController.getSimYaw()
    return _simYaw
end
