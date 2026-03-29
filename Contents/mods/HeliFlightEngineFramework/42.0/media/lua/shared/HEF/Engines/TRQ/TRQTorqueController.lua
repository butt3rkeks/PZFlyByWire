--[[
    TRQTorqueController — Hybrid angular PD controller with inertia normalization

    Yaw (Y):          scalar wrapAngle error — proven stable, no singularity
    Pitch/Roll (X/Z): tilt-decomposed quaternion error — immune to Euler ±90° flip

    Body→world error rotation: tilt error is body-frame, couple forces are world-frame.
    Omega from TRQAngularEstimator is body-frame (quaternion-based, no Euler gimbal).
    Hysteresis anti-unwinding: persistent h variable prevents inverted equilibrium.
    Gyroscopic feedforward: omega × (I * omega) cancels precession during yaw rotation.

    Depends only on Quaternion (Core model) + HeliConfig, not FBW modules.
    Torque output is in standard math convention (Y = up).
]]

TRQTorqueController = {}

local toLuaNum = HeliUtil.toLuaNum
local rad = math.rad
local acos = math.acos
local sqrt = math.sqrt
local abs = math.abs

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function wrapAngle(d)
    if d > 180 then d = d - 360
    elseif d < -180 then d = d + 360
    end
    return d
end

-------------------------------------------------------------------------------------
-- State
-------------------------------------------------------------------------------------
local _Ix = 1
local _Iy = 1
local _Iz = 1
local _inertiaValid = false

-- Hysteresis anti-unwinding (Mayhew, Sanfelice, Teel).
-- Persistent sign variable prevents quaternion unwinding at 180° tilt.
local _h = 1
local HYSTERESIS_DELTA = 0.2

-------------------------------------------------------------------------------------
-- Pre-allocated quaternions
-------------------------------------------------------------------------------------
local _actQ = Quaternion.new()

--- Inline fromEuler into pre-allocated quaternion.
local function setFromEuler(q, rx, ry, rz)
    local c1, c2, c3 = math.cos(rx/2), math.cos(ry/2), math.cos(rz/2)
    local s1, s2, s3 = math.sin(rx/2), math.sin(ry/2), math.sin(rz/2)
    q.w = c1*c2*c3 - s1*s2*s3
    q.x = s1*c2*c3 + c1*s2*s3
    q.y = c1*s2*c3 - s1*c2*s3
    q.z = c1*c2*s3 + s1*s2*c3
end

-------------------------------------------------------------------------------------
-- Vector3f reading (for inertia)
-------------------------------------------------------------------------------------

local function tryReadVec3(vec)
    if not vec then return nil, nil, nil end
    local ok, vx = pcall(function() return toLuaNum(vec:x()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:y()) end)
        local _, vz = pcall(function() return toLuaNum(vec:z()) end)
        return vx, vy or 0, vz or 0
    end
    ok, vx = pcall(function() return toLuaNum(vec:getX()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:getY()) end)
        local _, vz = pcall(function() return toLuaNum(vec:getZ()) end)
        return vx, vy or 0, vz or 0
    end
    local s = tostring(vec)
    if s then
        local a, b, c = s:match("([%d%.%-]+)[,%s]+([%d%.%-]+)[,%s]+([%d%.%-]+)")
        if a then return tonumber(a), tonumber(b), tonumber(c) end
    end
    return nil, nil, nil
end

-------------------------------------------------------------------------------------
-- Inertia computation
-------------------------------------------------------------------------------------

function TRQTorqueController.initFromVehicle(vehicle)
    if _inertiaValid then return end

    local mass = toLuaNum(vehicle:getMass())
    if mass <= 0 then _inertiaValid = false; return end

    local script = vehicle:getScript()
    if not script then _inertiaValid = false; return end

    local ex, ey, ez
    local ok, shape = pcall(function() return script:getPhysicsChassisShape() end)
    if ok and shape then ex, ey, ez = tryReadVec3(shape) end
    if not ex or (ex == 0 and ey == 0 and ez == 0) then
        ok, shape = pcall(function() return script:getExtents() end)
        if ok and shape then ex, ey, ez = tryReadVec3(shape) end
    end

    if ex and ey and ez and not (ex == 0 and ey == 0 and ez == 0) then
        _Ix = (mass / 12) * (ey * ey + ez * ez)
        _Iy = (mass / 12) * (ex * ex + ez * ez)
        _Iz = (mass / 12) * (ex * ex + ey * ey)
        print("[TRQ] Inertia from extents: Ix=" .. string.format("%.1f", _Ix)
            .. " Iy=" .. string.format("%.1f", _Iy) .. " Iz=" .. string.format("%.1f", _Iz))
    else
        local r = 1.5
        local I = 0.4 * mass * r * r
        _Ix = I; _Iy = I; _Iz = I
        print("[TRQ] WARNING: Could not read vehicle extents. Using fallback inertia: " .. string.format("%.1f", I))
    end

    if _Ix < 1 then _Ix = 1 end
    if _Iy < 1 then _Iy = 1 end
    if _Iz < 1 then _Iz = 1 end
    _inertiaValid = true
end

-------------------------------------------------------------------------------------
-- Hybrid PD controller
-------------------------------------------------------------------------------------

--- Compute torque vector.
--- @param desQuat table Desired orientation quaternion {w, x, y, z}
--- @param desYawDeg number Desired yaw scalar (degrees)
--- @param actAngleX number Actual Euler X (degrees, from Bullet)
--- @param actAngleY number Actual Euler Y (degrees, from Bullet)
--- @param actAngleZ number Actual Euler Z (degrees, from Bullet)
--- @param omegaX number Body-frame angular velocity X (deg/s, from quat estimator)
--- @param omegaY number Body-frame angular velocity Y (deg/s)
--- @param omegaZ number Body-frame angular velocity Z (deg/s)
--- @return number torqueX World-frame (standard math, Y=up)
--- @return number torqueY World-frame
--- @return number torqueZ World-frame
--- @return number angErrMag Error magnitude (rad)
function TRQTorqueController.compute(desQuat, desYawDeg,
                                     actAngleX, actAngleY, actAngleZ,
                                     omegaX, omegaY, omegaZ)

    -- === TILT ERROR (body-frame, heading-independent) ===

    -- Desired tilt = strip yaw from desired quaternion
    local desYawRad = rad(desYawDeg)
    local dyHalf = desYawRad / 2
    local cyw = math.cos(dyHalf)
    local cyy = -math.sin(dyHalf)  -- conjugate Y
    local dw, dx, dy, dz = desQuat.w, desQuat.x, desQuat.y, desQuat.z
    local dtw = cyw*dw - cyy*dy
    local dtx = cyw*dx + cyy*dz
    local dty = cyy*dw + cyw*dy
    local dtz = cyw*dz - cyy*dx

    -- Actual tilt = strip yaw from actual quaternion
    local actYawRad = rad(actAngleY)
    local ayHalf = actYawRad / 2
    local ayw = math.cos(ayHalf)
    local ayy = -math.sin(ayHalf)  -- conjugate Y
    setFromEuler(_actQ, rad(actAngleX), rad(actAngleY), rad(actAngleZ))
    local aw, ax, ay, az = _actQ.w, _actQ.x, _actQ.y, _actQ.z
    local atw = ayw*aw - ayy*ay
    local atx = ayw*ax + ayy*az
    local aty = ayy*aw + ayw*ay
    local atz = ayw*az - ayy*ax

    -- Tilt error = desTilt * conj(actTilt)
    local bw, bx, by, bz = atw, -atx, -aty, -atz
    local ew = dtw*bw - dtx*bx - dty*by - dtz*bz
    local ex = dtx*bw + dtw*bx + dty*bz - dtz*by
    local ey = dty*bw + dtw*by + dtz*bx - dtx*bz
    local ez = dtz*bw + dtw*bz + dtx*by - dty*bx

    -- Hysteresis anti-unwinding (Mayhew, Sanfelice, Teel).
    -- Persistent _h prevents unwinding: at 180° tilt, memoryless sign flip
    -- creates stable inverted equilibrium. Hysteresis holds the correction
    -- direction until the error clearly crosses the hemisphere boundary.
    if _h * ew < -HYSTERESIS_DELTA then
        _h = -_h
    end
    if _h < 0 then
        ew = -ew; ex = -ex; ey = -ey; ez = -ez
    end

    -- Axis-angle extraction
    local ewc = clamp(ew, -1, 1)
    local angle = 2 * acos(ewc)
    local errX, errZ = 0, 0
    local sinHalf = sqrt(1 - ewc * ewc)
    if sinHalf > 0.0001 then
        errX = (ex / sinHalf) * angle
        errZ = (ez / sinHalf) * angle
    end

    -- === YAW ERROR (scalar, proven stable) ===
    local errYDeg = wrapAngle(desYawDeg - actAngleY)
    local errY = rad(errYDeg)

    -- === ROTATE TILT ERROR: body → world frame ===
    local headingRad = rad(actAngleY)
    local cosH = math.cos(headingRad)
    local sinH = math.sin(headingRad)
    local worldErrX = errX * cosH - errZ * sinH
    local worldErrZ = errX * sinH + errZ * cosH

    local angErrMag = sqrt(worldErrX * worldErrX + errY * errY + worldErrZ * worldErrZ)

    -- === ROTATE OMEGA: body → world frame ===
    -- Omega from quaternion estimator is body-frame.
    local omegaXRad = rad(omegaX)
    local omegaYRad = rad(omegaY)
    local omegaZRad = rad(omegaZ)
    local worldOmegaXRad = omegaXRad * cosH - omegaZRad * sinH
    local worldOmegaZRad = omegaXRad * sinH + omegaZRad * cosH

    -- === PD TORQUE (world frame, per-axis inertia) ===
    local maxTorque = HeliConfig.GetTrqMaxTorque()

    local torqueX = _Ix * (HeliConfig.GetTrqPitchPGain() * worldErrX - HeliConfig.GetTrqPitchDGain() * worldOmegaXRad)
    local torqueY = _Iy * (HeliConfig.GetTrqYawPGain()   * errY      - HeliConfig.GetTrqYawDGain()   * omegaYRad)
    local torqueZ = _Iz * (HeliConfig.GetTrqRollPGain()  * worldErrZ - HeliConfig.GetTrqRollDGain()  * worldOmegaZRad)

    -- === GYROSCOPIC FEEDFORWARD (tunable, default OFF) ===
    -- Euler's equation: I*alpha = tau - omega × (I*omega)
    -- The cross-product term causes precession during yaw rotation.
    -- Adding it as feedforward cancels the disturbance.
    -- Scale 0=disabled, 1=full cancellation. Default 0 — the sign/magnitude
    -- interaction with Bullet's internal physics needs empirical tuning.
    local gyroScale = HeliConfig.GetTrqGyroScale()
    if gyroScale > 0 then
        -- Compute in body frame (where I is diagonal), then rotate to world.
        local Iox = _Ix * omegaXRad
        local Ioy = _Iy * omegaYRad
        local Ioz = _Iz * omegaZRad
        local gyroX = omegaYRad * Ioz - omegaZRad * Ioy
        local gyroY = omegaZRad * Iox - omegaXRad * Ioz
        local gyroZ = omegaXRad * Ioy - omegaYRad * Iox
        local worldGyroX = gyroX * cosH - gyroZ * sinH
        local worldGyroZ = gyroX * sinH + gyroZ * cosH

        torqueX = torqueX + gyroScale * worldGyroX
        torqueY = torqueY + gyroScale * gyroY
        torqueZ = torqueZ + gyroScale * worldGyroZ
    end

    torqueX = clamp(torqueX, -maxTorque, maxTorque)
    torqueY = clamp(torqueY, -maxTorque, maxTorque)
    torqueZ = clamp(torqueZ, -maxTorque, maxTorque)

    return torqueX, torqueY, torqueZ, angErrMag
end

--- @return number Ix, number Iy, number Iz, boolean valid
function TRQTorqueController.getInertia()
    return _Ix, _Iy, _Iz, _inertiaValid
end

function TRQTorqueController.reset()
    _Ix = 1; _Iy = 1; _Iz = 1
    _inertiaValid = false
    _h = 1
end
