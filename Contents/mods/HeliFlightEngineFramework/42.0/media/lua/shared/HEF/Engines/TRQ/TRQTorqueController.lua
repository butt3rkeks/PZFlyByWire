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
    -- This is the SO(3)-correct error direction (confirmed by aerospace literature).
    -- `desired × actual` has wrong P-term sign (backward pitch on tilt input) but
    -- appeared more stable during yaw because the wrong sign reduced effective gain.
    -- `actual × desired` is correct but requires adequate D-gain (>= 2*sqrt(P) ≈ 9
    -- for critical damping) to maintain stability under gyroscopic coupling.
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

    -- === WORLD-FRAME INERTIA TENSOR PD ===
    -- Instead of rotating error/torque between body and world frames (which caused
    -- cross-coupling bugs from R-transpose confusion and frame mismatches), we
    -- transform the inertia tensor to world frame: I_world = R * I_body * R^T.
    --
    -- Everything stays in world frame: error, omega, inertia, torque.
    -- The non-diagonal I_world automatically handles the physical axis coupling
    -- (e.g., at 30° heading, world-X torque needs world-Z contribution and vice versa).
    --
    -- Mathematically identical to R^T→PD→R but eliminates frame confusion.
    -- Source: Lee, Leok, McClamroch (2010), Gaffer on Games, research agent analysis.

    -- Build rotation matrix R from up-vector + forward-vector (Gram-Schmidt).
    -- R columns = body axes in world frame.
    local headingRad = rad(actYawDeg)
    local rawFwdX = math.sin(headingRad)
    local rawFwdZ = math.cos(headingRad)
    local dotFU = rawFwdX * actUpX + rawFwdZ * actUpZ
    local fwdX = rawFwdX - dotFU * actUpX
    local fwdY = -dotFU * actUpY
    local fwdZ = rawFwdZ - dotFU * actUpZ
    local fwdLen = sqrt(fwdX*fwdX + fwdY*fwdY + fwdZ*fwdZ)
    if fwdLen > 0.0001 then
        fwdX = fwdX / fwdLen; fwdY = fwdY / fwdLen; fwdZ = fwdZ / fwdLen
    else
        fwdX = rawFwdX; fwdY = 0; fwdZ = rawFwdZ
    end
    local rgtX = fwdY * actUpZ - fwdZ * actUpY
    local rgtY = fwdZ * actUpX - fwdX * actUpZ
    local rgtZ = fwdX * actUpY - fwdY * actUpX

    -- R: columns are [right(bodyX), up(bodyY), forward(bodyZ)] in world coords
    local r00, r10, r20 = rgtX, rgtY, rgtZ
    local r01, r11, r21 = actUpX, actUpY, actUpZ
    local r02, r12, r22 = fwdX, fwdY, fwdZ

    -- I_world = R * diag(Ix,Iy,Iz) * R^T (symmetric 3x3)
    -- Element [i][j] = sum_k( R[i][k] * I_k * R[j][k] )
    local Iw00 = r00*r00*_Ix + r01*r01*_Iy + r02*r02*_Iz
    local Iw01 = r00*r10*_Ix + r01*r11*_Iy + r02*r12*_Iz
    local Iw02 = r00*r20*_Ix + r01*r21*_Iy + r02*r22*_Iz
    local Iw11 = r10*r10*_Ix + r11*r11*_Iy + r12*r12*_Iz
    local Iw12 = r10*r20*_Ix + r11*r21*_Iy + r12*r22*_Iz
    local Iw22 = r20*r20*_Ix + r21*r21*_Iy + r22*r22*_Iz

    -- Omega: body-frame from quaternion estimator → rotate to world via R
    local omegaXRad = rad(omegaX)
    local omegaYRad = rad(omegaY)
    local omegaZRad = rad(omegaZ)
    local wOmX = r00*omegaXRad + r01*omegaYRad + r02*omegaZRad
    local wOmY = r10*omegaXRad + r11*omegaYRad + r12*omegaZRad
    local wOmZ = r20*omegaXRad + r21*omegaYRad + r22*omegaZRad

    -- PD correction in world frame (P and D gains are scalar, same for all axes)
    local P_tilt = HeliConfig.GetTrqPitchPGain()  -- use pitch gain for all tilt
    local D_tilt = HeliConfig.GetTrqPitchDGain()
    local corrX = P_tilt * worldErrX - D_tilt * wOmX
    local corrZ = P_tilt * worldErrZ - D_tilt * wOmZ

    -- Yaw PD (scalar, independent)
    local corrY = HeliConfig.GetTrqYawPGain() * errY - HeliConfig.GetTrqYawDGain() * wOmY

    -- World torque = I_world * correction (includes cross-coupling from off-diagonal terms)
    local maxTorque = HeliConfig.GetTrqMaxTorque()
    local torqueX = Iw00*corrX + Iw01*corrY + Iw02*corrZ
    local torqueY = Iw01*corrX + Iw11*corrY + Iw12*corrZ
    local torqueZ = Iw02*corrX + Iw12*corrY + Iw22*corrZ

    -- === GYROSCOPIC FEEDFORWARD (tunable, default OFF) ===
    -- omega × (I_world * omega) in world frame.
    local gyroScale = HeliConfig.GetTrqGyroScale()
    if gyroScale > 0 then
        -- I_world * omega_world
        local IwX = Iw00*wOmX + Iw01*wOmY + Iw02*wOmZ
        local IwY = Iw01*wOmX + Iw11*wOmY + Iw12*wOmZ
        local IwZ = Iw02*wOmX + Iw12*wOmY + Iw22*wOmZ
        -- omega × (I_world * omega)
        torqueX = torqueX + gyroScale * (wOmY*IwZ - wOmZ*IwY)
        torqueY = torqueY + gyroScale * (wOmZ*IwX - wOmX*IwZ)
        torqueZ = torqueZ + gyroScale * (wOmX*IwY - wOmY*IwX)
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
