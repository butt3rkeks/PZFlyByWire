--[[
    TRQYawController — Yaw heading management for torque-based engine

    Key difference from FBWYawController:
      - FBW snaps simYaw to actual heading on key release ("reanchor"). This creates
        a step change in desired heading that the teleport-based FBW handles instantly
        but TRQ's torque PD cannot — it causes a sudden yaw error spike.
      - TRQ: on key release, desired heading freezes at current desired value (NOT
        actual). The PD's D-term naturally decelerates yaw rotation. No reanchor snap.
      - Heading reanchor for sim position (checkHeadingReanchor) still works —
        it detects when the desired heading has changed significantly, triggering
        sim position snap to prevent horizontal drift during yaw.

    Pure controller logic. Reads HeliConfig for threshold scaling.
]]

TRQYawController = {}

-------------------------------------------------------------------------------------
-- State
-------------------------------------------------------------------------------------
local _desiredYawDeg = nil
local _lastDesiredYaw = nil

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset all yaw state.
function TRQYawController.reset()
    _desiredYawDeg = nil
    _lastDesiredYaw = nil
end

--- Update yaw tracking.
--- During rotation: advance desired heading by yaw delta.
--- On release: freeze desired heading at current desired value (no snap to actual).
--- @param currentYawDeg number Current actual yaw (degrees, from vehicle)
--- @param isRotating boolean Whether A/D keys are pressed
--- @param yawDelta number Yaw change this frame (degrees)
--- @return number desiredYawDeg The desired heading for the PD controller
function TRQYawController.update(currentYawDeg, isRotating, yawDelta)
    if not _desiredYawDeg then
        -- First call: initialize from actual heading
        _desiredYawDeg = currentYawDeg
    end

    if isRotating then
        -- Advance desired heading at input rate
        _desiredYawDeg = _desiredYawDeg + yawDelta
        -- Wrap to [-180, 180]
        if _desiredYawDeg > 180 then _desiredYawDeg = _desiredYawDeg - 360
        elseif _desiredYawDeg < -180 then _desiredYawDeg = _desiredYawDeg + 360
        end
    end
    -- On release: desired heading stays where it is — PD decelerates naturally.
    -- No snap to actual heading (the key difference from FBW).

    return _desiredYawDeg
end

--- Check if heading has changed enough to warrant a sim position re-anchor.
--- Same threshold logic as FBW — detects large heading changes for horizontal
--- sim correction. This is about the SIM POSITION, not yaw control.
--- @param fps number Current average FPS
--- @return boolean shouldReanchor
function TRQYawController.checkHeadingReanchor(fps)
    if not _desiredYawDeg or not _lastDesiredYaw then
        _lastDesiredYaw = _desiredYawDeg
        return false
    end

    local yawDelta = math.abs(_desiredYawDeg - _lastDesiredYaw)
    if yawDelta > 180 then yawDelta = 360 - yawDelta end

    local reanchorThreshold = 2.0 * HeliConfig.TARGET_FPS / math.max(fps, HeliConfig.MIN_FPS)
    _lastDesiredYaw = _desiredYawDeg

    return yawDelta > reanchorThreshold
end

--- Get current desired heading.
--- @return number|nil Desired yaw in degrees
function TRQYawController.getYaw()
    return _desiredYawDeg
end

--- Alias for interface compatibility with code expecting getSimYaw.
function TRQYawController.getSimYaw()
    return _desiredYawDeg
end
