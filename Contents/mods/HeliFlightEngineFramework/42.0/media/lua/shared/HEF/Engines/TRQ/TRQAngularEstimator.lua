--[[
    TRQAngularEstimator — Quaternion-based angular velocity estimation

    Computes body-frame angular velocity from frame-to-frame quaternion difference.
    Immune to Euler gimbal lock (no Euler angle differencing).

    Method: omega_body = (2/dt) * (conj(q_prev) * q_current).xyz
    Source: Mario Garcia (mariogc.com), Fabian Giesen (fgiesen.wordpress.com)
]]

TRQAngularEstimator = {}

local _prevQ = nil  -- previous frame's quaternion {w, x, y, z}
local _omegaX = 0   -- smoothed body-frame omega (rad/s)
local _omegaY = 0
local _omegaZ = 0

--- Inline fromEuler (same as TRQTorqueController — avoids allocation)
local function quatFromEuler(rx, ry, rz)
    local c1, c2, c3 = math.cos(rx/2), math.cos(ry/2), math.cos(rz/2)
    local s1, s2, s3 = math.sin(rx/2), math.sin(ry/2), math.sin(rz/2)
    return {
        w = c1*c2*c3 - s1*s2*s3,
        x = s1*c2*c3 + c1*s2*s3,
        y = c1*s2*c3 - s1*c2*s3,
        z = c1*c2*s3 + s1*s2*c3,
    }
end

--- Update angular velocity estimate from current Euler angles.
--- Builds quaternion internally (avoids Euler differencing gimbal lock).
--- Returns body-frame angular velocity in DEGREES/second (for compatibility
--- with existing D-term code).
--- @param angleX number Current Euler X (degrees)
--- @param angleY number Current Euler Y (degrees)
--- @param angleZ number Current Euler Z (degrees)
--- @param dt number Frame time (seconds)
--- @return number omegaX Body-frame angular velocity (deg/s, EMA-smoothed)
--- @return number omegaY Body-frame angular velocity (deg/s, EMA-smoothed)
--- @return number omegaZ Body-frame angular velocity (deg/s, EMA-smoothed)
function TRQAngularEstimator.update(angleX, angleY, angleZ, dt)
    local curQ = quatFromEuler(math.rad(angleX), math.rad(angleY), math.rad(angleZ))

    if not _prevQ or dt <= 0 then
        _prevQ = curQ
        return 0, 0, 0
    end

    -- q_delta = conj(q_prev) * q_current (body-frame rotation this frame)
    local pw, px, py, pz = _prevQ.w, -_prevQ.x, -_prevQ.y, -_prevQ.z  -- conjugate
    local cw, cx, cy, cz = curQ.w, curQ.x, curQ.y, curQ.z

    local dw = pw*cw - px*cx - py*cy - pz*cz
    local dx = px*cw + pw*cx + py*cz - pz*cy
    local dy = py*cw + pw*cy + pz*cx - px*cz
    local dz = pz*cw + pw*cz + px*cy - py*cx

    -- Ensure shortest path (avoid 360°-path omega spike)
    if dw < 0 then
        dx = -dx; dy = -dy; dz = -dz
    end

    -- First-order omega approximation: omega = (2/dt) * q_delta.xyz (rad/s)
    local scale = 2.0 / dt
    local rawOmegaX = dx * scale
    local rawOmegaY = dy * scale
    local rawOmegaZ = dz * scale

    -- Convert to deg/s for compatibility
    local deg = math.deg
    rawOmegaX = deg(rawOmegaX)
    rawOmegaY = deg(rawOmegaY)
    rawOmegaZ = deg(rawOmegaZ)

    -- EMA smoothing
    local alpha = HeliConfig.GetTrqOmegaAlpha()
    _omegaX = alpha * rawOmegaX + (1 - alpha) * _omegaX
    _omegaY = alpha * rawOmegaY + (1 - alpha) * _omegaY
    _omegaZ = alpha * rawOmegaZ + (1 - alpha) * _omegaZ

    _prevQ = curQ

    return _omegaX, _omegaY, _omegaZ
end

--- Reset all state. Called on flight reset / vehicle exit.
function TRQAngularEstimator.reset()
    _prevQ = nil
    _omegaX = 0
    _omegaY = 0
    _omegaZ = 0
end
