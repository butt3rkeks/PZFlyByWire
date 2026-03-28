--[[
    TRQAngularEstimator — Angular velocity estimation from Euler angle differences

    No getAngularVelocity() API in PZ, so we estimate from frame-to-frame
    Euler angle changes with EMA smoothing.
]]

TRQAngularEstimator = {}

local _prevAngleX = nil
local _prevAngleY = nil
local _prevAngleZ = nil
local _omegaX = 0
local _omegaY = 0
local _omegaZ = 0

--- Wrap angle delta to [-180, +180] to handle discontinuity at ±180°.
--- @param d number Raw angle difference (degrees)
--- @return number Wrapped difference
local function wrapAngle(d)
    -- Fast path: most deltas are small
    if d > 180 then d = d - 360
    elseif d < -180 then d = d + 360
    end
    return d
end

--- Update angular velocity estimate from current Euler angles.
--- @param angleX number Current Euler X (degrees)
--- @param angleY number Current Euler Y (degrees)
--- @param angleZ number Current Euler Z (degrees)
--- @param dt number Frame time (seconds)
--- @return number omegaX Angular velocity around X (deg/s, EMA-smoothed)
--- @return number omegaY Angular velocity around Y (deg/s, EMA-smoothed)
--- @return number omegaZ Angular velocity around Z (deg/s, EMA-smoothed)
function TRQAngularEstimator.update(angleX, angleY, angleZ, dt)
    if _prevAngleX == nil or dt <= 0 then
        -- First frame: store angles, return zero
        _prevAngleX = angleX
        _prevAngleY = angleY
        _prevAngleZ = angleZ
        return 0, 0, 0
    end

    local rawOmegaX = wrapAngle(angleX - _prevAngleX) / dt
    local rawOmegaY = wrapAngle(angleY - _prevAngleY) / dt
    local rawOmegaZ = wrapAngle(angleZ - _prevAngleZ) / dt

    local alpha = HeliConfig.GetTrqOmegaAlpha()
    _omegaX = alpha * rawOmegaX + (1 - alpha) * _omegaX
    _omegaY = alpha * rawOmegaY + (1 - alpha) * _omegaY
    _omegaZ = alpha * rawOmegaZ + (1 - alpha) * _omegaZ

    _prevAngleX = angleX
    _prevAngleY = angleY
    _prevAngleZ = angleZ

    return _omegaX, _omegaY, _omegaZ
end

--- Reset all state. Called on flight reset / vehicle exit.
function TRQAngularEstimator.reset()
    _prevAngleX = nil
    _prevAngleY = nil
    _prevAngleZ = nil
    _omegaX = 0
    _omegaY = 0
    _omegaZ = 0
end
