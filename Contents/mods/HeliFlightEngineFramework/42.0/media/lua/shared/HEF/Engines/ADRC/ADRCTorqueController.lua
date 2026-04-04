--[[
    ADRCTorqueController -- Cascaded quaternion attitude controller

    Architecture (ArduPilot/PX4 pattern):
      Outer loop: quaternion error -> body-frame rate setpoints (P controller)
      Inner loop: per-axis rate ADRC -> body-frame torque (2nd-order ESO)

    The outer loop uses quaternion error (q_err = q_des * conj(q_act)) to
    produce COUPLED rate setpoints. This avoids the cross-axis coupling that
    broke independent per-axis position ADRC with asymmetric inertia.

    The inner loop tracks angular rate error with a 1st-order plant model
    (rate' = b*u + f). Body inertia is CONSTANT (no heading dependency).
    Per-axis rate control is physically valid because angular rates are independent.

    ESO uses two-phase exact discretization for substep timing.
    Analytical inertia from physicsChassisShape.
]]

ADRCTorqueController = {}

local toLuaNum = HeliUtil.toLuaNum
local rad = math.rad
local cos = math.cos
local sin = math.sin
local sqrt = math.sqrt
local abs = math.abs
local exp = math.exp

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

-------------------------------------------------------------------------------------
-- State
-------------------------------------------------------------------------------------
local _Ix = 1   -- body pitch inertia
local _Iy = 1   -- body yaw inertia
local _Iz = 1   -- body roll inertia
local _inertiaValid = false
local _mass = 1

-- 2nd-order rate ESO state per body axis: {x1 = rate_error, x2 = disturbance}
local _esoPitch = {x1 = 0, x2 = 0}
local _esoRoll  = {x1 = 0, x2 = 0}
local _esoYaw   = {x1 = 0, x2 = 0}

-- Last frame's applied torque per body axis
local _uPrevPitch = 0
local _uPrevRoll  = 0
local _uPrevYaw   = 0

local _esoInitialized = false

-- Previous frame's quaternion for angular velocity measurement
local _prevQw, _prevQx, _prevQy, _prevQz = nil, nil, nil, nil

-- ESO bandwidth warmup ramp
local _woRampFrame = 0

-- Diagnostic pulse state
local _diagFrameCount = 0
local _diagPrevUpX = nil
local _diagPrevUpY = nil
local _diagPrevUpZ = nil
local _diagPrevYawRad = nil

-------------------------------------------------------------------------------------
-- Rate ESO: 2nd-order, two-phase exact discretization
-------------------------------------------------------------------------------------

--- Bullet fixed substep duration (seconds).
local DT_SUBSTEP = 0.01

--- Two-phase exact ESO for 1st-order plant (rate' = -b*u + f).
--- Phase 1: predict with control active for dt_s = 0.01s (one Bullet substep).
--- Phase 2: predict with zero control for (T - dt_s).
--- Discrete observer gains place double pole at z = e^(-wo*T).
---
--- @param eso table ESO state {x1, x2}
--- @param y number Measurement (rate error, rad/s)
--- @param b number Control effectiveness (1/I_body)
--- @param u_prev number Previous control output (torque Nm)
--- @param wo number Observer bandwidth (rad/s)
--- @param dt number Frame time (seconds)
local function rateEsoUpdate(eso, y, b, u_prev, wo, dt)
    local T = dt
    if T < 0.001 then T = 0.001 end

    -- Phase 1: predict with control active (dt_s seconds)
    local ds = DT_SUBSTEP
    if ds > T then ds = T end
    local bup = b * u_prev
    local x1_1 = eso.x1 + (-bup + eso.x2) * ds
    local x2_1 = eso.x2

    -- Phase 2: predict with zero control (T - dt_s seconds)
    local T2 = T - ds
    if T2 > 0 then
        x1_1 = x1_1 + x2_1 * T2
    end

    -- Discrete observer gains: double pole at z = e^(-wo*T)
    local a = exp(-wo * T)
    local K1 = 1 - a * a
    local K2 = (1 - a) * (1 - a) / T

    -- Measurement correction
    local e = y - x1_1
    eso.x1 = x1_1 + K1 * e
    eso.x2 = x2_1 + K2 * e

    -- Leaky disturbance decay (prevents phantom accumulation)
    eso.x2 = eso.x2 * 0.97
end

--- Rate ADRC control law: u = (x2 + wc * x1) / b
--- 1st-order: proportional on rate error (x1) + disturbance rejection (x2).
--- @param eso table ESO state {x1, x2}
--- @param wc number Controller bandwidth (rad/s)
--- @param b number Control effectiveness (1/I_body)
--- @return number torque (Nm)
local function rateAdrcControl(eso, wc, b)
    return (eso.x2 + wc * eso.x1) / b
end

-------------------------------------------------------------------------------------
-- Vector3f reading (for inertia computation from vehicle script)
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
        local va, vb, vc = s:match("([%d%.%-]+)[,%s]+([%d%.%-]+)[,%s]+([%d%.%-]+)")
        if va then return tonumber(va), tonumber(vb), tonumber(vc) end
    end
    return nil, nil, nil
end

-------------------------------------------------------------------------------------
-- Inertia computation from vehicle script
-------------------------------------------------------------------------------------

--- Compute body-frame inertia tensor from physicsChassisShape (box approximation).
--- Called once per vehicle. API returns post-scaled values (modelScale already applied).
--- @param vehicle BaseVehicle
function ADRCTorqueController.initFromVehicle(vehicle)
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

    local corrFactor = HeliConfig.GetAdrcInertiaCorrFactor()

    if ex and ey and ez and not (ex == 0 and ey == 0 and ez == 0) then
        _Ix = (mass / 12) * (ey * ey + ez * ez) * corrFactor
        _Iy = (mass / 12) * (ex * ex + ez * ez) * corrFactor
        _Iz = (mass / 12) * (ex * ex + ey * ey) * corrFactor
        print("[ADRC] Box inertia (x" .. string.format("%.2f", corrFactor) .. "): Ix="
            .. string.format("%.1f", _Ix) .. " Iy=" .. string.format("%.1f", _Iy)
            .. " Iz=" .. string.format("%.1f", _Iz)
            .. " mass=" .. string.format("%.0f", mass))
    else
        local r = 1.5
        local I = 0.4 * mass * r * r * corrFactor
        _Ix = I; _Iy = I; _Iz = I
        print("[ADRC] WARNING: No extents. Fallback inertia: " .. string.format("%.1f", I))
    end

    _mass = mass
    _inertiaValid = true
end

-------------------------------------------------------------------------------------
-- Cascaded attitude controller
-------------------------------------------------------------------------------------

--- Compute body-frame torque using cascaded quaternion -> rate ADRC.
--- @param q_des table Desired quaternion {w, x, y, z}
--- @param q_act table Actual quaternion {w, x, y, z}
--- @param dt number Frame time (seconds)
--- @param subSteps number Bullet substeps this frame
--- @return number bodyTorquePitch, bodyTorqueYaw, bodyTorqueRoll
--- @return number angErrMag
--- @return number desYawDeg, actYawDeg, errYDeg
--- @return number esoRatePitch, esoRateYaw, esoRateRoll (ESO x1 = rate error)
--- @return number Ix, Iz
--- @return number esoDistPitch, esoDistRoll, esoDistYaw (ESO x2 = disturbance)
--- @return number rateSpPitch, rateSpRoll
--- @return number wo
function ADRCTorqueController.compute(q_des, q_act, dt, subSteps)

    -- === QUATERNION ERROR: q_err = q_des * conj(q_act) ===
    -- This is the rotation FROM actual TO desired, expressed in body frame.
    local aw, ax, ay, az = q_act.w, q_act.x, q_act.y, q_act.z
    local dw, dx, dy, dz = q_des.w, q_des.x, q_des.y, q_des.z
    -- conj(q_act) = (aw, -ax, -ay, -az)
    -- q_err = q_des * conj(q_act)
    local caw, cax, cay, caz = aw, -ax, -ay, -az
    local ew = dw*caw - dx*cax - dy*cay - dz*caz
    local ex = dx*caw + dw*cax + dy*caz - dz*cay
    local ey = dy*caw + dw*cay + dz*cax - dx*caz
    local ez = dz*caw + dw*caz + dx*cay - dy*cax

    -- Shortest path: ensure w >= 0
    if ew < 0 then ew = -ew; ex = -ex; ey = -ey; ez = -ez end

    -- === RATE SETPOINTS from quaternion error ===
    -- 2*vec(q_err) ~= error angle (rad) for small errors, bounded at 2.0 for 180deg.
    -- Kp_att scales error angle to desired angular rate (rad/s).
    local Kp_att = HeliConfig.GetAdrcKpAtt()
    local Kp_yaw = HeliConfig.GetAdrcWcYaw()  -- reuse yaw bandwidth as yaw attitude gain
    local rateSpPitch = Kp_att * 2 * ex   -- body-X rate setpoint (rad/s)
    local rateSpRoll  = Kp_att * 2 * ez   -- body-Z rate setpoint (rad/s)
    local rateSpYaw   = Kp_yaw * 2 * ey   -- body-Y rate setpoint (rad/s)

    local angErrMag = 2 * sqrt(ex*ex + ey*ey + ez*ez)

    -- === ANGULAR VELOCITY MEASUREMENT (quaternion differencing) ===
    -- omega_body = (2/dt) * vec(conj(q_prev) * q_act)
    local measPitchRate, measRollRate, measYawRate = 0, 0, 0
    if _prevQw and dt > 0.001 then
        -- conj(q_prev) * q_act
        local pw, px, py, pz = _prevQw, -_prevQx, -_prevQy, -_prevQz
        local qdw = pw*aw - px*ax - py*ay - pz*az
        local qdx = px*aw + pw*ax + py*az - pz*ay
        local qdy = py*aw + pw*ay + pz*ax - px*az
        local qdz = pz*aw + pw*az + px*ay - py*ax
        -- Shortest path
        if qdw < 0 then qdx = -qdx; qdy = -qdy; qdz = -qdz end
        local scale = 2.0 / dt
        measPitchRate = qdx * scale  -- body omega X (rad/s)
        measYawRate   = qdy * scale  -- body omega Y (rad/s)
        measRollRate  = qdz * scale  -- body omega Z (rad/s)
    end
    _prevQw = aw; _prevQx = ax; _prevQy = ay; _prevQz = az

    -- === RATE ERROR ===
    local rateErrPitch = rateSpPitch - measPitchRate
    local rateErrRoll  = rateSpRoll  - measRollRate
    local rateErrYaw   = rateSpYaw   - measYawRate

    -- === BODY-FRAME INERTIA: constant ===
    local b_pitch = 1 / _Ix
    local b_roll  = 1 / _Iz
    local b_yaw   = 1 / _Iy

    -- === PHYSICS TIMESTEP ===
    local nSteps = math.max(subSteps or 1, 1)
    local physicsDt = nSteps * DT_SUBSTEP

    -- === ESO BANDWIDTH RAMP ===
    if _woRampFrame >= 0 then
        _woRampFrame = _woRampFrame + 1
    end
    local wo = HeliConfig.GetAdrcEsoWo()
    local warmupTotal = HeliConfig.GetAdrcWarmupFrames()
    if _woRampFrame >= 0 and _woRampFrame <= warmupTotal then
        local progress = _woRampFrame / warmupTotal
        local woFrac = HeliConfig.GetAdrcWarmupWoFraction()
        wo = wo * (woFrac + (1.0 - woFrac) * progress)
    end
    local wcPitch = HeliConfig.GetAdrcWcTilt()  -- pitch rate ADRC bandwidth
    local wcRoll  = HeliConfig.GetAdrcWcRoll()  -- roll rate ADRC bandwidth (lower for low-inertia axis)
    local wcYawRate = HeliConfig.GetAdrcWcYaw()
    local maxTorque = HeliConfig.GetAdrcMaxTorque()

    -- === INITIALIZE ESOs ===
    if not _esoInitialized then
        _esoPitch.x1 = rateErrPitch; _esoPitch.x2 = 0
        _esoRoll.x1  = rateErrRoll;  _esoRoll.x2 = 0
        _esoYaw.x1   = rateErrYaw;   _esoYaw.x2 = 0
        _esoInitialized = true
    end

    -- === UPDATE RATE ESOs ===
    rateEsoUpdate(_esoPitch, rateErrPitch, b_pitch, _uPrevPitch, wo, physicsDt)
    rateEsoUpdate(_esoRoll,  rateErrRoll,  b_roll,  _uPrevRoll,  wo, physicsDt)
    rateEsoUpdate(_esoYaw,   rateErrYaw,   b_yaw,   _uPrevYaw,   wo, physicsDt)

    -- === RATE ADRC CONTROL LAW ===
    local rawPitch = rateAdrcControl(_esoPitch, wcPitch, b_pitch)
    local rawRoll  = rateAdrcControl(_esoRoll,  wcRoll,  b_roll)
    local rawYaw   = rateAdrcControl(_esoYaw, wcYawRate, b_yaw)

    -- === PRIORITY ALLOCATION ===
    local tiltDemand = abs(rawPitch) + abs(rawRoll)
    local yawDemand = abs(rawYaw)
    local torquePitch = rawPitch
    local torqueRoll = rawRoll
    local torqueYaw = rawYaw

    if tiltDemand + yawDemand > maxTorque then
        if tiltDemand <= maxTorque then
            local yawBudget = maxTorque - tiltDemand
            torqueYaw = rawYaw * (yawBudget / yawDemand)
        else
            local tiltScale = maxTorque / tiltDemand
            torquePitch = rawPitch * tiltScale
            torqueRoll = rawRoll * tiltScale
            torqueYaw = 0
        end
    end

    -- === STORE APPLIED TORQUES ===
    _uPrevPitch = torquePitch
    _uPrevRoll = torqueRoll
    _uPrevYaw = torqueYaw

    -- === RETURN ===
    -- Yaw debug values from quaternion
    local desFwdX = 2 * (q_des.x * q_des.z + q_des.w * q_des.y)
    local desFwdZ = 1 - 2 * (q_des.x * q_des.x + q_des.y * q_des.y)
    local desYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))
    local actFwdX = 2 * (ax * az + aw * ay)
    local actFwdZ = 1 - 2 * (ax * ax + ay * ay)
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))
    local errYDeg = desYawDeg - actYawDeg
    if errYDeg > 180 then errYDeg = errYDeg - 360
    elseif errYDeg < -180 then errYDeg = errYDeg + 360 end

    return torquePitch, torqueYaw, torqueRoll, angErrMag,
           desYawDeg, actYawDeg, errYDeg,
           _esoPitch.x1, _esoYaw.x1, _esoRoll.x1,  -- rate errors (ESO x1)
           _Ix, _Iz,
           _esoPitch.x2, _esoRoll.x2, _esoYaw.x2,   -- disturbances (ESO x2)
           rateSpPitch, rateSpRoll,
           wo
end

--- @return number Ix, number Iy, number Iz, boolean valid
function ADRCTorqueController.getInertia()
    return _Ix, _Iy, _Iz, _inertiaValid
end

--- Signal that the helicopter has lifted off. Resets ESO states and starts
--- the bandwidth ramp. Called by ADRCEngine on first airborne frame.
function ADRCTorqueController.notifyLiftoff()
    _woRampFrame = 0
    _esoPitch = {x1 = 0, x2 = 0}
    _esoRoll  = {x1 = 0, x2 = 0}
    _esoYaw   = {x1 = 0, x2 = 0}
    _uPrevPitch = 0; _uPrevRoll = 0; _uPrevYaw = 0
    _esoInitialized = false
    -- Preserve _prevQ for angular velocity measurement across liftoff
end

function ADRCTorqueController.reset()
    _Ix = 1; _Iy = 1; _Iz = 1
    _inertiaValid = false
    _esoPitch = {x1 = 0, x2 = 0}
    _esoRoll  = {x1 = 0, x2 = 0}
    _esoYaw   = {x1 = 0, x2 = 0}
    _uPrevPitch = 0; _uPrevRoll = 0; _uPrevYaw = 0
    _esoInitialized = false
    _prevQw = nil; _prevQx = nil; _prevQy = nil; _prevQz = nil
    _woRampFrame = -1
    _diagFrameCount = 0
    _diagPrevUpX = nil; _diagPrevUpY = nil; _diagPrevUpZ = nil
    _diagPrevYawRad = nil
end
