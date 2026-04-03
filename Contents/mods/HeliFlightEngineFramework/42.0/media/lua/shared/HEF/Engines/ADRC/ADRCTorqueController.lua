--[[
    ADRCTorqueController -- Body-frame ADRC angular controller with exact ESO

    Operates in BODY frame: pitch (body-X/right), roll (body-Z/forward), yaw (body-Y/up).
    Each axis has a 3rd-order ESO with CONSTANT b = 1/I_body.

    ESO uses two-phase exact discretization (research_fps_independence.md Method 1):
      Phase 1: predict with control active for dt_s=0.01s (one Bullet substep)
      Phase 2: predict with zero control for (T - dt_s)
      Correct: discrete observer gains place triple pole at z = e^(-wo*T)
    This eliminates the timing mismatch artifact where the ESO absorbs 1/N
    substep error as false disturbance. Unconditionally stable at any FPS.

    Analytical inertia from physicsChassisShape (box inertia formula).
    ESO bandwidth ramp at startup (warmup period).

    Depends only on HeliConfig (via ADRCHeliConfig getters).
    Torque output is body-frame (X=pitch, Y=yaw, Z=roll).
]]

ADRCTorqueController = {}

local toLuaNum = HeliUtil.toLuaNum
local rad = math.rad
local acos = math.acos
local sqrt = math.sqrt
local abs = math.abs
local exp = math.exp

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
local _Ix = 1   -- body pitch inertia
local _Iy = 1   -- body yaw inertia
local _Iz = 1   -- body roll inertia
local _inertiaValid = false
local _mass = 1

-- ESO state per body axis
local _esoPitch = {x1 = 0, x2 = 0, x3 = 0}
local _esoRoll  = {x1 = 0, x2 = 0, x3 = 0}
local _esoYaw   = {x1 = 0, x2 = 0, x3 = 0}

-- Last frame's applied torque per body axis (after allocation)
local _uPrevPitch = 0
local _uPrevRoll  = 0
local _uPrevYaw   = 0

local _esoInitialized = false

-- Internal warmup: ramps ESO bandwidth from low to full over N frames.
-- Lives here (not in ADRCEngine) so ALL callers (update + updateGround) get the ramp.
local _woRampFrame = 0

-- Diagnostic pulse state
local _diagFrameCount = 0
local _diagPrevUpX = nil
local _diagPrevUpY = nil
local _diagPrevUpZ = nil
local _diagPrevYawRad = nil

-------------------------------------------------------------------------------------
-- ESO core: Two-phase exact discretization
-------------------------------------------------------------------------------------

--- Bullet fixed substep duration (seconds).
local DT_SUBSTEP = 0.01

--- Two-phase exact ESO update with predictor-corrector architecture.
---
--- Models the actual physics timing: torque active for dt_s=0.01s (one Bullet
--- substep), then zero for (T - dt_s). The plant is a triple integrator
--- (y'' = b*u + f), so the free prediction is exact polynomial.
---
--- Discrete observer gains place a triple pole at z = e^(-wo*T).
--- Unconditionally stable (no Euler dt*wo limit, no sub-stepping needed).
---
--- @param eso table ESO state {x1, x2, x3}
--- @param y number Measurement (angular error, radians)
--- @param b number Control effectiveness (1/I_body)
--- @param u_prev number Previous control output (torque Nm)
--- @param wo number Observer bandwidth (rad/s)
--- @param dt number Frame time (seconds)
--- @param maxDist number|nil Optional disturbance clamp
local function esoUpdate(eso, y, b, u_prev, wo, dt, maxDist)
    local T = dt
    if T < 0.001 then T = 0.001 end

    -- Phase 1: predict with control active (dt_s seconds)
    local ds = DT_SUBSTEP
    if ds > T then ds = T end  -- at very high FPS, dt < 0.01
    local ds2 = ds * ds
    local bup = b * u_prev
    local x1_1 = eso.x1 + eso.x2 * ds + eso.x3 * ds2 / 2 - bup * ds2 / 2
    local x2_1 = eso.x2 + eso.x3 * ds - bup * ds
    local x3_1 = eso.x3

    -- Phase 2: predict with zero control (T - dt_s seconds)
    local T2 = T - ds
    if T2 > 0 then
        local T2sq = T2 * T2
        x1_1 = x1_1 + x2_1 * T2 + x3_1 * T2sq / 2
        x2_1 = x2_1 + x3_1 * T2
        -- x3 unchanged (constant disturbance model)
    end

    -- Discrete observer gains: triple pole at z = e^(-wo*T)
    local a = exp(-wo * T)
    local oma = 1 - a
    local K1 = 3 * oma
    local K2 = oma * oma * (5 + a) / (2 * T)
    local K3 = oma * oma * oma / (T * T)

    -- Measurement correction
    local e = y - x1_1
    eso.x1 = x1_1 + K1 * e
    eso.x2 = x2_1 + K2 * e
    eso.x3 = x3_1 + K3 * e

    if maxDist then
        eso.x3 = clamp(eso.x3, -maxDist, maxDist)
    end
end

--- ADRC control law: u = (x3 + wc^2 * x1 + 2*wc * x2) / b
--- Combines proportional (x1), derivative (x2), and disturbance rejection (x3).
--- @param eso table ESO state
--- @param wc number Controller bandwidth (rad/s)
--- @param b number Control effectiveness (1/I_body)
--- @return number torque (Nm)
local function adrcControl(eso, wc, b)
    local wc2 = wc * wc
    return (eso.x3 + wc2 * eso.x1 + 2 * wc * eso.x2) / b
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
        -- Box inertia from extents (script coords: X=lateral, Y=height, Z=length)
        _Ix = (mass / 12) * (ey * ey + ez * ez) * corrFactor
        _Iy = (mass / 12) * (ex * ex + ez * ez) * corrFactor
        _Iz = (mass / 12) * (ex * ex + ey * ey) * corrFactor
        print("[ADRC] Box inertia (x" .. string.format("%.2f", corrFactor) .. "): Ix="
            .. string.format("%.1f", _Ix) .. " Iy=" .. string.format("%.1f", _Iy)
            .. " Iz=" .. string.format("%.1f", _Iz)
            .. " mass=" .. string.format("%.0f", mass))
    else
        -- Fallback: uniform sphere
        local r = 1.5
        local I = 0.4 * mass * r * r * corrFactor
        _Ix = I; _Iy = I; _Iz = I
        print("[ADRC] WARNING: No extents. Fallback inertia: " .. string.format("%.1f", I))
    end

    _mass = mass
    _inertiaValid = true
end

-------------------------------------------------------------------------------------
-- Body-frame ADRC controller
-------------------------------------------------------------------------------------

--- Compute body-frame torque using ADRC with two-phase exact ESO.
--- @param desUpX number Desired up-vector X (Bullet coords: X=east, Y=up, Z=north)
--- @param desUpY number Desired up-vector Y
--- @param desUpZ number Desired up-vector Z
--- @param desYawDeg number Desired yaw (degrees)
--- @param actUpX number Actual up-vector X
--- @param actUpY number Actual up-vector Y
--- @param actUpZ number Actual up-vector Z
--- @param actYawDeg number Actual yaw (degrees)
--- @param rightX number Body right-axis X (Bullet world coords)
--- @param rightY number Body right-axis Y
--- @param rightZ number Body right-axis Z
--- @param fwdX number Body forward-axis X
--- @param fwdY number Body forward-axis Y
--- @param fwdZ number Body forward-axis Z
--- @param dt number Frame time (seconds)
--- @param subSteps number Bullet substeps this frame
--- @return number bodyTorqueX (pitch)
--- @return number bodyTorqueY (yaw)
--- @return number bodyTorqueZ (roll)
--- @return number angErrMag
--- @return number desYawDeg, number actYawDeg, number errYDeg
--- @return number esoRatePitch, number esoRateYaw, number esoRateRoll
--- @return number Ix, number Iz
--- @return number esoDistPitch, number esoDistRoll, number esoDistYaw
--- @return number esoErrPitch, number esoErrRoll
function ADRCTorqueController.compute(desUpX, desUpY, desUpZ, desYawDeg,
                                      actUpX, actUpY, actUpZ, actYawDeg,
                                      rightX, rightY, rightZ,
                                      fwdX, fwdY, fwdZ,
                                      dt, subSteps)

    -- === DIAGNOSTIC PULSE MODE ===
    local diagAxis = HeliConfig.GetAdrcDiagPulseAxis()
    if diagAxis and diagAxis > 0 then
        _diagFrameCount = _diagFrameCount + 1
        local DIAG_DELAY = 300

        if _diagFrameCount > DIAG_DELAY then
            local nSteps = math.max(subSteps or 1, 1)
            local physicsDt = nSteps * 0.01
            local pulseTorque = HeliConfig.GetAdrcDiagPulseTorque()

            -- Measure body-frame angular rates from up-vector change
            local measPitchRate, measRollRate, measYawRate = 0, 0, 0
            if _diagPrevUpX and physicsDt > 0 then
                local dupx = actUpX - _diagPrevUpX
                local dupy = actUpY - _diagPrevUpY
                local dupz = actUpZ - _diagPrevUpZ
                measPitchRate = (dupx * fwdX + dupy * fwdY + dupz * fwdZ) / physicsDt
                measRollRate = (dupx * rightX + dupy * rightY + dupz * rightZ) / physicsDt
                local yawRad = rad(actYawDeg)
                if _diagPrevYawRad then
                    local dyaw = yawRad - _diagPrevYawRad
                    if dyaw > 3.14159 then dyaw = dyaw - 6.28318
                    elseif dyaw < -3.14159 then dyaw = dyaw + 6.28318 end
                    measYawRate = dyaw / physicsDt
                end
                _diagPrevYawRad = yawRad
            end
            _diagPrevUpX = actUpX; _diagPrevUpY = actUpY; _diagPrevUpZ = actUpZ

            -- Pulse: 30 frames on, 30 frames off
            local elapsed = _diagFrameCount - DIAG_DELAY
            local pulseOn = ((elapsed % 60) < 30)
            local tP, tR, tY = 0, 0, 0
            if pulseOn then
                if diagAxis == 1 then tP = pulseTorque
                elseif diagAxis == 2 then tR = pulseTorque
                elseif diagAxis == 3 then tY = pulseTorque end
            end

            return tP, tY, tR, 0,
                   desYawDeg, actYawDeg, 0,
                   measPitchRate, measYawRate, measRollRate,
                   _Ix, _Iz,
                   measPitchRate, measRollRate,
                   pulseOn and 1 or 0, pulseTorque
        end
        -- During delay: fall through to normal ADRC
    end

    -- === TILT ERROR via up-vector cross product (world frame) ===
    local crossX = actUpY * desUpZ - actUpZ * desUpY
    local crossY = actUpZ * desUpX - actUpX * desUpZ
    local crossZ = actUpX * desUpY - actUpY * desUpX

    local dotProd = clamp(actUpX * desUpX + actUpY * desUpY + actUpZ * desUpZ, -1, 1)
    local tiltAngle = acos(dotProd)
    local sinAngle = sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ)

    local worldErrX, worldErrY, worldErrZ = 0, 0, 0
    if sinAngle > 0.0001 then
        local scale = tiltAngle / sinAngle
        worldErrX = crossX * scale
        worldErrY = crossY * scale
        worldErrZ = crossZ * scale
    end

    -- === PROJECT WORLD ERROR ONTO BODY AXES ===
    local bodyErrPitch = worldErrX * rightX + worldErrY * rightY + worldErrZ * rightZ
    local bodyErrRoll  = worldErrX * fwdX  + worldErrY * fwdY  + worldErrZ * fwdZ

    -- === YAW ERROR (scalar) ===
    local errYDeg = wrapAngle(desYawDeg - actYawDeg)
    local errYaw = rad(errYDeg)

    local angErrMag = sqrt(bodyErrPitch * bodyErrPitch + errYaw * errYaw + bodyErrRoll * bodyErrRoll)

    -- === BODY-FRAME INERTIA ===
    local I_pitch = _Ix
    local I_roll = _Iz
    local I_yaw = _Iy

    -- === PHYSICS TIMESTEP ===
    local nSteps = math.max(subSteps or 1, 1)
    local physicsDt = nSteps * DT_SUBSTEP

    -- === ESO BANDWIDTH RAMP ===
    -- Ramp restarts on liftoff (notifyLiftoff resets _woRampFrame).
    -- During ground mode, uses full wo (ground errors are small, no risk).
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
    local wcTilt = HeliConfig.GetAdrcWcTilt()
    local wcYaw = HeliConfig.GetAdrcWcYaw()
    local maxTorque = HeliConfig.GetAdrcMaxTorque()

    local b_pitch = 1 / I_pitch
    local b_roll = 1 / I_roll
    local b_yaw = 1 / I_yaw

    -- === INITIALIZE ESOs on first frame ===
    if not _esoInitialized then
        _esoPitch.x1 = bodyErrPitch; _esoPitch.x2 = 0; _esoPitch.x3 = 0
        _esoRoll.x1  = bodyErrRoll;  _esoRoll.x2 = 0;  _esoRoll.x3 = 0
        _esoYaw.x1   = errYaw;       _esoYaw.x2 = 0;   _esoYaw.x3 = 0
        _esoInitialized = true
    end

    -- === UPDATE ESOs ===
    local tiltDisabled = (wcTilt == 0)
    local maxDistPitch = maxTorque * b_pitch
    local maxDistRoll = maxTorque * b_roll
    local maxDistYaw = maxTorque * b_yaw
    if not tiltDisabled then
        esoUpdate(_esoPitch, bodyErrPitch, b_pitch, _uPrevPitch, wo, physicsDt, maxDistPitch)
        esoUpdate(_esoRoll,  bodyErrRoll,  b_roll,  _uPrevRoll,  wo, physicsDt, maxDistRoll)
    end
    esoUpdate(_esoYaw, errYaw, b_yaw, _uPrevYaw, wo, physicsDt, maxDistYaw)

    -- === ADRC CONTROL LAW ===
    local rawPitch = tiltDisabled and 0 or adrcControl(_esoPitch, wcTilt, b_pitch)
    local rawRoll  = tiltDisabled and 0 or adrcControl(_esoRoll,  wcTilt, b_roll)
    local rawYaw   = adrcControl(_esoYaw, wcYaw, b_yaw)

    -- === PRIORITY-BASED TORQUE ALLOCATION (tilt over yaw) ===
    local tiltDemand = abs(rawPitch) + abs(rawRoll)
    local yawDemand = abs(rawYaw)
    local totalDemand = tiltDemand + yawDemand

    local torquePitch, torqueYaw, torqueRoll

    if totalDemand <= maxTorque then
        torquePitch = rawPitch
        torqueYaw = rawYaw
        torqueRoll = rawRoll
    elseif tiltDemand <= maxTorque then
        torquePitch = rawPitch
        torqueRoll = rawRoll
        local yawBudget = maxTorque - tiltDemand
        local yawScale = yawBudget / yawDemand
        torqueYaw = rawYaw * yawScale
    else
        local tiltScale = maxTorque / tiltDemand
        torquePitch = rawPitch * tiltScale
        torqueRoll = rawRoll * tiltScale
        torqueYaw = 0
    end

    -- === GYROSCOPIC FEEDFORWARD (optional) ===
    local gyroScale = HeliConfig.GetAdrcGyroScale()
    if gyroScale > 0 then
        local wP = _esoPitch.x2
        local wY = _esoYaw.x2
        local wR = _esoRoll.x2
        local IwP = I_pitch * wP
        local IwY = I_yaw * wY
        local IwR = I_roll * wR
        torquePitch = torquePitch + gyroScale * (wY * IwR - wR * IwY)
        torqueYaw   = torqueYaw   + gyroScale * (wR * IwP - wP * IwR)
        torqueRoll  = torqueRoll  + gyroScale * (wP * IwY - wY * IwP)
    end

    -- === STORE APPLIED TORQUES for next frame's ESO ===
    _uPrevPitch = torquePitch
    _uPrevRoll = torqueRoll
    _uPrevYaw = torqueYaw

    -- === RETURN: body torques + diagnostics ===
    return torquePitch, torqueYaw, torqueRoll, angErrMag,
           desYawDeg, actYawDeg, errYDeg,
           _esoPitch.x2, _esoYaw.x2, _esoRoll.x2,
           I_pitch, I_roll,
           _esoPitch.x3, _esoRoll.x3, _esoYaw.x3,
           _esoPitch.x1, _esoRoll.x1,
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
    -- Re-initialize ESO from scratch on liftoff
    _esoPitch = {x1 = 0, x2 = 0, x3 = 0}
    _esoRoll  = {x1 = 0, x2 = 0, x3 = 0}
    _esoYaw   = {x1 = 0, x2 = 0, x3 = 0}
    _uPrevPitch = 0; _uPrevRoll = 0; _uPrevYaw = 0
    _esoInitialized = false
end

function ADRCTorqueController.reset()
    _Ix = 1; _Iy = 1; _Iz = 1
    _inertiaValid = false
    _esoPitch = {x1 = 0, x2 = 0, x3 = 0}
    _esoRoll  = {x1 = 0, x2 = 0, x3 = 0}
    _esoYaw   = {x1 = 0, x2 = 0, x3 = 0}
    _uPrevPitch = 0; _uPrevRoll = 0; _uPrevYaw = 0
    _esoInitialized = false
    _woRampFrame = -1  -- disabled until notifyLiftoff
    _diagFrameCount = 0
    _diagPrevUpX = nil; _diagPrevUpY = nil; _diagPrevUpZ = nil
    _diagPrevYawRad = nil
end
