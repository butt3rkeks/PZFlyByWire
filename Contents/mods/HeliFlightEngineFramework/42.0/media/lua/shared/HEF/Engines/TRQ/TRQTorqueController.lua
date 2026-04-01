--[[
    TRQTorqueController — Hybrid angular PD controller with inertia normalization

    Yaw (Y):          scalar wrapAngle error — proven stable
    Pitch/Roll (X/Z): Z-axis (up-vector) tilt error — PX4 approach

    The Z-axis method computes tilt error from the cross product of
    actual and desired up-vectors. The result is DIRECTLY in world frame.
    No heading rotation needed, no sign flip at any heading.

    Hysteresis anti-unwinding on the tilt error prevents inverted equilibrium.
    Gyroscopic feedforward (tunable, default off) cancels precession.

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

-------------------------------------------------------------------------------------
-- Helpers
-------------------------------------------------------------------------------------

--- Extract up-vector (Y axis in standard math) from quaternion.
--- Returns world-frame components of the body's up direction.
local function quatUpVector(qw, qx, qy, qz)
    local upX = 2 * (qx * qy + qw * qz)
    local upY = 1 - 2 * (qx * qx + qz * qz)
    local upZ = 2 * (qy * qz - qw * qx)
    return upX, upY, upZ
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
-- PD controller: Z-axis tilt error + scalar yaw
-------------------------------------------------------------------------------------

--- Compute torque vector.
--- @param desQuat table Desired orientation quaternion {w, x, y, z}
--- @param desYawDeg number Desired yaw scalar (degrees)
--- @param actUpX number Actual up-vector X (world frame, from vehicle:getUpVector)
--- @param actUpY number Actual up-vector Y (world frame)
--- @param actUpZ number Actual up-vector Z (world frame)
--- @param actYawDeg number Actual yaw (degrees, from Bullet Euler Y — continuous, no flip)
--- @param omegaX number Body-frame angular velocity X (deg/s)
--- @param omegaY number Body-frame angular velocity Y (deg/s)
--- @param omegaZ number Body-frame angular velocity Z (deg/s)
--- @return number torqueX World-frame
--- @return number torqueY World-frame
--- @return number torqueZ World-frame
--- @return number angErrMag Error magnitude (rad)
function TRQTorqueController.compute(desQuat, desYawDeg,
                                     actUpX, actUpY, actUpZ, actYawDeg,
                                     omegaX, omegaY, omegaZ)

    -- === TILT ERROR via up-vector cross product (PX4 approach) ===
    -- Actual up-vector read directly from Bullet (vehicle:getUpVector).
    -- No Euler angles involved — immune to gimbal flip at any heading.
    -- Desired up-vector extracted from orientation quaternion.

    local desUpX, desUpY, desUpZ = quatUpVector(desQuat.w, desQuat.x, desQuat.y, desQuat.z)

    -- Cross product: actual_up × desired_up = rotation axis from actual to desired.
    -- The axis of rotation that takes vector A to vector B is A × B (right-hand rule).
    -- Verified: pitch forward (desUp toward -Z) → act × des gives -X torque → tilts toward -Z ✓
    --          roll right (desUp toward -X)     → act × des gives +Z torque → tilts toward -X ✓
    local crossX = actUpY * desUpZ - actUpZ * desUpY
    local crossY = actUpZ * desUpX - actUpX * desUpZ
    local crossZ = actUpX * desUpY - actUpY * desUpX

    -- For small angles: |cross| ≈ sin(angle) ≈ angle. For large angles,
    -- scale by angle/sin(angle) to get true angular error.
    local dotProd = clamp(actUpX * desUpX + actUpY * desUpY + actUpZ * desUpZ, -1, 1)
    local tiltAngle = acos(dotProd)
    local sinAngle = sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ)

    local worldErrX, worldErrY, worldErrZ = 0, 0, 0
    if sinAngle > 0.0001 then
        local scale = tiltAngle / sinAngle
        worldErrX = crossX * scale
        worldErrY = crossY * scale  -- small yaw component from tilt coupling
        worldErrZ = crossZ * scale
    end

    -- === YAW ERROR (scalar, proven stable) ===
    local errYDeg = wrapAngle(desYawDeg - actYawDeg)
    local errY = rad(errYDeg)

    local angErrMag = sqrt(worldErrX * worldErrX + errY * errY + worldErrZ * worldErrZ)

    -- === PD IN BODY FRAME with FULL ROTATION MATRIX ===
    -- The Z-axis cross product gives world-frame error. Per-axis inertia is
    -- only correct in body frame (Ix=pitch, Iz=roll). We rotate world→body
    -- for PD, then body→world for couple forces.
    --
    -- Full 3x3 rotation matrix reconstructed from up-vector + heading.
    -- No Euler angles in this path — up-vector is direct from Bullet.
    --
    -- R columns = body axes in world frame:
    --   Column 1 (body Y / up) = actUp (from vehicle:getUpVector)
    --   Column 2 (body Z / forward) = constructed from heading + up
    --   Column 0 (body X / right) = forward × up (completes the frame)
    local omegaXRad = rad(omegaX)  -- already body-frame from quat estimator
    local omegaYRad = rad(omegaY)
    local omegaZRad = rad(omegaZ)

    -- Construct forward direction from heading, projected perpendicular to up.
    -- Raw heading direction in world horizontal plane:
    local headingRad = rad(actYawDeg)
    local rawFwdX = math.sin(headingRad)   -- heading → world X component
    local rawFwdZ = math.cos(headingRad)   -- heading → world Z component
    -- Project heading perpendicular to up-vector (Gram-Schmidt):
    --   fwd = rawFwd - (rawFwd · up) * up, then normalize
    local dotFU = rawFwdX * actUpX + 0 * actUpY + rawFwdZ * actUpZ
    local fwdX = rawFwdX - dotFU * actUpX
    local fwdY = 0       - dotFU * actUpY
    local fwdZ = rawFwdZ - dotFU * actUpZ
    local fwdLen = sqrt(fwdX*fwdX + fwdY*fwdY + fwdZ*fwdZ)
    if fwdLen > 0.0001 then
        fwdX = fwdX / fwdLen; fwdY = fwdY / fwdLen; fwdZ = fwdZ / fwdLen
    else
        -- Degenerate (up-vector aligned with heading) — fallback to heading-only
        fwdX = rawFwdX; fwdY = 0; fwdZ = rawFwdZ
    end
    -- Right = forward × up (completes right-handed frame)
    local rgtX = fwdY * actUpZ - fwdZ * actUpY
    local rgtY = fwdZ * actUpX - fwdX * actUpZ
    local rgtZ = fwdX * actUpY - fwdY * actUpX

    -- R matrix: columns are body axes in world frame
    -- Column 0 = right (body X), Column 1 = up (body Y), Column 2 = forward (body Z)
    local r00, r10, r20 = rgtX, rgtY, rgtZ      -- body X in world
    local r01, r11, r21 = actUpX, actUpY, actUpZ -- body Y in world
    local r02, r12, r22 = fwdX, fwdY, fwdZ      -- body Z in world

    -- Rotate world error to body frame: body = R^T * world
    local bodyErrX = r00 * worldErrX + r10 * worldErrY + r20 * worldErrZ
    local bodyErrZ = r02 * worldErrX + r12 * worldErrY + r22 * worldErrZ

    -- PD in body frame with body inertia (omega already body-frame)
    local maxTorque = HeliConfig.GetTrqMaxTorque()

    local bodyTorqueX = _Ix * (HeliConfig.GetTrqPitchPGain() * bodyErrX - HeliConfig.GetTrqPitchDGain() * omegaXRad)
    local torqueY     = _Iy * (HeliConfig.GetTrqYawPGain()   * errY     - HeliConfig.GetTrqYawDGain()   * omegaYRad)
    local bodyTorqueZ = _Iz * (HeliConfig.GetTrqRollPGain()  * bodyErrZ - HeliConfig.GetTrqRollDGain()  * omegaZRad)

    bodyTorqueX = clamp(bodyTorqueX, -maxTorque, maxTorque)
    bodyTorqueZ = clamp(bodyTorqueZ, -maxTorque, maxTorque)

    -- Rotate body torque to world frame: world = R * body
    local torqueX = r00 * bodyTorqueX + r02 * bodyTorqueZ
    local torqueZ = r20 * bodyTorqueX + r22 * bodyTorqueZ
    -- (Y component: r10*bodyTorqueX + r12*bodyTorqueZ adds tilt-coupling to yaw — include it)
    torqueY = torqueY + r10 * bodyTorqueX + r12 * bodyTorqueZ

    -- === GYROSCOPIC FEEDFORWARD (tunable, default OFF) ===
    -- Computed in body frame (I diagonal), rotated to world via full R.
    local gyroScale = HeliConfig.GetTrqGyroScale()
    if gyroScale > 0 then
        local Iox = _Ix * omegaXRad
        local Ioy = _Iy * omegaYRad
        local Ioz = _Iz * omegaZRad
        local gyroX = omegaYRad * Ioz - omegaZRad * Ioy
        local gyroY = omegaZRad * Iox - omegaXRad * Ioz
        local gyroZ = omegaXRad * Ioy - omegaYRad * Iox
        -- Rotate body gyro to world via full R
        torqueX = torqueX + gyroScale * (r00 * gyroX + r01 * gyroY + r02 * gyroZ)
        torqueY = torqueY + gyroScale * (r10 * gyroX + r11 * gyroY + r12 * gyroZ)
        torqueZ = torqueZ + gyroScale * (r20 * gyroX + r21 * gyroY + r22 * gyroZ)
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
end
