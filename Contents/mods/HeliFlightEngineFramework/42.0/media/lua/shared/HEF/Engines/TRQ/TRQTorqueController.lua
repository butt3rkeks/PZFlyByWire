--[[
    TRQTorqueController — Angular PD controller with inertia normalization

    Computes torque to drive actual orientation toward desired (from FBWOrientation).
    Normalizes by analytically computed box inertia so PD gains are vehicle-independent.

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

-------------------------------------------------------------------------------------
-- Inertia state (computed once per vehicle)
-------------------------------------------------------------------------------------
local _Ix = 1  -- moment of inertia around X (pitch)
local _Iy = 1  -- moment of inertia around Y (yaw)
local _Iz = 1  -- moment of inertia around Z (roll)
local _inertiaValid = false

-------------------------------------------------------------------------------------
-- Pre-allocated quaternions for zero-alloc hot path
-------------------------------------------------------------------------------------
local _desiredQ = Quaternion.new()
local _actualQ  = Quaternion.new()
local _errQ     = Quaternion.new()
local _conjQ    = Quaternion.new()

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Try to read Vector3f components. Kahlua may not expose Vector3f methods
--- (library class, not explicitly setExposed). Tries x()/getX()/tostring parse.
--- @param vec Java Vector3f
--- @return number|nil x, number|nil y, number|nil z
local function tryReadVec3(vec)
    if not vec then return nil, nil, nil end
    -- Try JOML-style x() method
    local ok, vx = pcall(function() return toLuaNum(vec:x()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:y()) end)
        local _, vz = pcall(function() return toLuaNum(vec:z()) end)
        return vx, vy or 0, vz or 0
    end
    -- Try JavaBean-style getX() method
    ok, vx = pcall(function() return toLuaNum(vec:getX()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:getY()) end)
        local _, vz = pcall(function() return toLuaNum(vec:getZ()) end)
        return vx, vy or 0, vz or 0
    end
    -- Try tostring parse: "Vector3f[1.500, 2.000, 3.000]" or "(1.5, 2.0, 3.0)"
    local s = tostring(vec)
    if s then
        local a, b, c = s:match("([%d%.%-]+)[,%s]+([%d%.%-]+)[,%s]+([%d%.%-]+)")
        if a then return tonumber(a), tonumber(b), tonumber(c) end
    end
    return nil, nil, nil
end

--- Compute box inertia from vehicle script extents and mass.
--- Falls back to mass × default radius² if Vector3f is not readable.
--- Called once during initFlight().
--- @param vehicle BaseVehicle
function TRQTorqueController.initFromVehicle(vehicle)
    if _inertiaValid then return end  -- already computed for this vehicle

    local mass = toLuaNum(vehicle:getMass())
    if mass <= 0 then
        _inertiaValid = false
        return
    end

    local script = vehicle:getScript()
    if not script then
        _inertiaValid = false
        return
    end

    -- Try physicsChassisShape first (what Bullet uses), then extents as fallback.
    -- Avoid hasPhysicsChassisShape() — Java Boolean may not unbox in Kahlua.
    local ex, ey, ez
    local ok, shape = pcall(function() return script:getPhysicsChassisShape() end)
    if ok and shape then
        ex, ey, ez = tryReadVec3(shape)
    end
    if not ex or (ex == 0 and ey == 0 and ez == 0) then
        ok, shape = pcall(function() return script:getExtents() end)
        if ok and shape then
            ex, ey, ez = tryReadVec3(shape)
        end
    end

    if ex and ey and ez and not (ex == 0 and ey == 0 and ez == 0) then
        -- Box inertia: I_axis = (m/12)(other1² + other2²)
        -- Half-extents in Bullet coords: X=east, Y=up, Z=north
        _Ix = (mass / 12) * (ey * ey + ez * ez)  -- pitch (around X)
        _Iy = (mass / 12) * (ex * ex + ez * ez)  -- yaw (around Y)
        _Iz = (mass / 12) * (ex * ex + ey * ey)  -- roll (around Z)
        print("[TRQ] Inertia from extents: Ix=" .. string.format("%.1f", _Ix)
            .. " Iy=" .. string.format("%.1f", _Iy) .. " Iz=" .. string.format("%.1f", _Iz))
    else
        -- Fallback: treat vehicle as uniform sphere with radius ~1.5m
        -- I = (2/5) * mass * r² (sphere approximation)
        local r = 1.5
        local I = 0.4 * mass * r * r
        _Ix = I
        _Iy = I
        _Iz = I
        print("[TRQ] WARNING: Could not read vehicle extents. Using fallback inertia: " .. string.format("%.1f", I))
    end

    -- Safety floor
    if _Ix < 1 then _Ix = 1 end
    if _Iy < 1 then _Iy = 1 end
    if _Iz < 1 then _Iz = 1 end

    _inertiaValid = true
end

--- Compute torque vector from angular error + angular velocity.
--- Desired orientation is read from FBWOrientation.toEuler().
--- @param actualAngleX number Current vehicle Euler X (degrees)
--- @param actualAngleY number Current vehicle Euler Y (degrees)
--- @param actualAngleZ number Current vehicle Euler Z (degrees)
--- @param omegaX number Angular velocity around X (deg/s from estimator)
--- @param omegaY number Angular velocity around Y (deg/s from estimator)
--- @param omegaZ number Angular velocity around Z (deg/s from estimator)
--- @return number torqueX Standard math convention (Y=up)
--- @return number torqueY Standard math convention (Y=up)
--- @return number torqueZ Standard math convention (Y=up)
--- @return number angErrMag Angular error magnitude (rad, for debug)
function TRQTorqueController.compute(actualAngleX, actualAngleY, actualAngleZ,
                                     omegaX, omegaY, omegaZ)
    -- 1. Build desired quaternion from FBWOrientation
    local desExDeg, desEyDeg, desEzDeg = FBWOrientation.toEuler()
    -- fromEuler takes radians
    local desRx, desRy, desRz = rad(desExDeg), rad(desEyDeg), rad(desEzDeg)
    local actRx, actRy, actRz = rad(actualAngleX), rad(actualAngleY), rad(actualAngleZ)

    -- Build quaternions (zero-alloc: reuse pre-allocated instances)
    -- Quaternion.fromEuler allocates, so we use set + manual construction
    -- Actually fromEuler returns a new table. For the hot path, inline:
    local c1, c2, c3 = math.cos(desRx/2), math.cos(desRy/2), math.cos(desRz/2)
    local s1, s2, s3 = math.sin(desRx/2), math.sin(desRy/2), math.sin(desRz/2)
    _desiredQ.w = c1*c2*c3 - s1*s2*s3
    _desiredQ.x = s1*c2*c3 + c1*s2*s3
    _desiredQ.y = c1*s2*c3 - s1*c2*s3
    _desiredQ.z = c1*c2*s3 + s1*s2*c3

    c1, c2, c3 = math.cos(actRx/2), math.cos(actRy/2), math.cos(actRz/2)
    s1, s2, s3 = math.sin(actRx/2), math.sin(actRy/2), math.sin(actRz/2)
    _actualQ.w = c1*c2*c3 - s1*s2*s3
    _actualQ.x = s1*c2*c3 + c1*s2*s3
    _actualQ.y = c1*s2*c3 - s1*c2*s3
    _actualQ.z = c1*c2*s3 + s1*s2*c3

    -- 2. Error quaternion: q_err = desired * conjugate(actual)
    _conjQ.w = _actualQ.w
    _conjQ.x = -_actualQ.x
    _conjQ.y = -_actualQ.y
    _conjQ.z = -_actualQ.z

    local qw = _desiredQ.w
    local qx = _desiredQ.x
    local qy = _desiredQ.y
    local qz = _desiredQ.z
    local rw = _conjQ.w
    local rx = _conjQ.x
    local ry = _conjQ.y
    local rz = _conjQ.z

    _errQ.w = qw*rw - qx*rx - qy*ry - qz*rz
    _errQ.x = qx*rw + qw*rx + qy*rz - qz*ry
    _errQ.y = qy*rw + qw*ry + qz*rx - qx*rz
    _errQ.z = qz*rw + qw*rz + qx*ry - qy*rx

    -- 3. Shortest path: negate if w < 0
    if _errQ.w < 0 then
        _errQ.w = -_errQ.w
        _errQ.x = -_errQ.x
        _errQ.y = -_errQ.y
        _errQ.z = -_errQ.z
    end

    -- 4. Extract axis-angle error
    local ew = clamp(_errQ.w, -1, 1)
    local angle = 2 * acos(ew)  -- total angular error in radians

    local errX, errY, errZ = 0, 0, 0
    local sinHalf = sqrt(1 - ew * ew)
    if sinHalf > 0.0001 then
        errX = (_errQ.x / sinHalf) * angle
        errY = (_errQ.y / sinHalf) * angle
        errZ = (_errQ.z / sinHalf) * angle
    end

    -- 5. PD torque: torque = I * (P * err - D * omega_rad)
    local maxTorque = HeliConfig.GetTrqMaxTorque()

    local omegaXRad = rad(omegaX)
    local omegaYRad = rad(omegaY)
    local omegaZRad = rad(omegaZ)

    local torqueX = _Ix * (HeliConfig.GetTrqPitchPGain() * errX - HeliConfig.GetTrqPitchDGain() * omegaXRad)
    local torqueY = _Iy * (HeliConfig.GetTrqYawPGain()   * errY - HeliConfig.GetTrqYawDGain()   * omegaYRad)
    local torqueZ = _Iz * (HeliConfig.GetTrqRollPGain()  * errZ - HeliConfig.GetTrqRollDGain()  * omegaZRad)

    torqueX = clamp(torqueX, -maxTorque, maxTorque)
    torqueY = clamp(torqueY, -maxTorque, maxTorque)
    torqueZ = clamp(torqueZ, -maxTorque, maxTorque)

    return torqueX, torqueY, torqueZ, angle
end

--- Get computed inertia values (for debug display).
--- @return number Ix, number Iy, number Iz, boolean valid
function TRQTorqueController.getInertia()
    return _Ix, _Iy, _Iz, _inertiaValid
end

--- Reset state.
function TRQTorqueController.reset()
    _Ix = 1
    _Iy = 1
    _Iz = 1
    _inertiaValid = false
end
