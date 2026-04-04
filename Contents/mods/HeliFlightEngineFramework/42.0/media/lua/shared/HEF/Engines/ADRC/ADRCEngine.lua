--[[
    ADRCEngine -- Clean cascaded-loop flight engine implementing IFlightEngine

    Physics-based horizontal control: tilt creates horizontal acceleration
    naturally via gravity + thrust decomposition. NO simulation model, NO
    correction forces, NO error tracker. This is the ArduPilot Stabilize
    mode architecture: stick -> tilt -> ADRC torque controller -> Bullet.

    Vertical control: same as framework (FlightModel + ForceComputer thrust).
    Velocity damping (drag model) provides clean stops when near-level.

    Fully self-contained: uses Core/ modules + ADRC-specific modules.
    Zero FBW/TRQ dependencies.

    Registers itself at file scope: IFlightEngine.register("ADRC", ADRCEngine)
]]

ADRCEngine = {}

-- Vertical velocity smoothing and adaptive gain constants
ADRCEngine.VERTICAL_VELOCITY_SMOOTHING = 0.3
ADRCEngine.ADAPTIVE_GAIN_ALPHA   = 0.05
ADRCEngine.ADAPTIVE_GAIN_MIN     = 1.0
ADRCEngine.ADAPTIVE_GAIN_MAX     = 8.0
ADRCEngine.ADAPTIVE_GAIN_DEADZONE = 0.3

-------------------------------------------------------------------------------------
-- Engine state
-------------------------------------------------------------------------------------
local function wrapAngle(d)
    if d > 180 then d = d - 360
    elseif d < -180 then d = d + 360
    end
    return d
end

--- Sanitize a number for CSV output. Kahlua's string.format crashes on NaN/Inf.
local function safeNum(v)
    if v ~= v then return 0 end          -- NaN check (NaN ~= NaN)
    if v == math.huge then return 1e15 end
    if v == -math.huge then return -1e15 end
    return v
end

local _flightAssistOff = false
local _warmupCounter = 0
local _tireInflationSet = false
local _airborneStarted = false

-- Smoothed values
local _gravTrim = 0
local _smoothedVelY = 0
local _smoothedYawRate = 0
local _smoothedPitchRate = 0
local _smoothedRollRate = 0
local _adaptiveGainMultiplier = 1.0
local _rampedTargetVelY = 0

-- Yaw controller state (inline, no separate module needed for ADRC)
local _desiredYawDeg = nil
local _yawCoasting = false
local _yawDampTorque = 0
local _prevActYawDeg = nil
local _esoWarmupFrame = 0

-- Debug state packed into one table (Kahlua 60-upvalue limit)
local _dbg = {
    torqueX=0, torqueY=0, torqueZ=0,
    angErrMag=0,
    targetVelY=0,
    actUpX=0, actUpY=0, actUpZ=0,
    desUpX=0, desUpY=0, desUpZ=0,
    desAngleX=0, desAngleY=0, desAngleZ=0,
    actAngleX=0, actAngleY=0, actAngleZ=0,
    ctrlDesYaw=0, ctrlActYaw=0, ctrlErrY=0,
    esoRatePitch=0, esoRateYaw=0, esoRateRoll=0,
    Ix=0, Iz=0,
    esoDistPitch=0, esoDistRoll=0,
    esoErrPitch=0, esoErrRoll=0,
    esoDistYaw=0, woEffective=0, mass=0,
    vertForce=0,
    pitchDelta=0, rollDelta=0, yawLead=0,
    dragFX=0, dragFZ=0,
    hSpeed=0, displaySpeed=0,
}

-------------------------------------------------------------------------------------
-- IFlightEngine: Metadata
-------------------------------------------------------------------------------------

function ADRCEngine.getInfo()
    return {
        name = "ADRC",
        version = "0.1",
        description = "Cascaded-loop ADRC (no sim model, physics-based horizontal)"
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Lifecycle
-------------------------------------------------------------------------------------

function ADRCEngine.resetFlightState()
    _flightAssistOff = false
    _warmupCounter = HeliConfig.GetAdrcWarmupFrames()
    _gravTrim = 0
    _smoothedVelY = 0
    _smoothedYawRate = 0
    _smoothedPitchRate = 0
    _smoothedRollRate = 0
    _rampedTargetVelY = 0
    _adaptiveGainMultiplier = 1.0
    _desiredYawDeg = nil
    _yawCoasting = false
    _yawDampTorque = 0
    _prevActYawDeg = nil
    _esoWarmupFrame = 0
    _airborneStarted = false
    _tireInflationSet = false

    ADRCOrientation.reset()
    ADRCTorqueController.reset()
    HeliForceAdapter.resetPhysicsTime()
    HeliVelocityAdapter.resetSmoothing()

    -- Reset debug state
    for k, _ in pairs(_dbg) do _dbg[k] = 0 end
end

function ADRCEngine.initFlight(vehicle)
    ADRCTorqueController.initFromVehicle(vehicle)
    _tireInflationSet = false
end

function ADRCEngine.tickWarmup()
    if _warmupCounter > 0 then
        _warmupCounter = _warmupCounter - 1
    end
end

function ADRCEngine.isWarmedUp()
    return _warmupCounter <= 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: update(ctx) -- one frame of airborne flight
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFUpdateResult
function ADRCEngine.update(ctx)
    local vehicle = ctx.vehicle
    local keys = ctx.keys
    local mass = ctx.mass

    ADRCTorqueController.initFromVehicle(vehicle)

    -- First airborne frame: reset ESO and start bandwidth ramp
    if not _airborneStarted then
        ADRCTorqueController.notifyLiftoff()
        _airborneStarted = true
    end

    local freeMode = vehicle:getModData().AutoBalance == true
    _flightAssistOff = freeMode

    -- Set phantom wheel tire inflation (once, deferred from initFlight)
    if not _tireInflationSet then
        local ok = pcall(vehicle.setTireInflation, vehicle, 0, 1.0)
        if ok then _tireInflationSet = true end
    end

    -- 1. Init orientation from vehicle if needed
    if not ADRCOrientation.isInitialized() then
        ADRCOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- 1b. Read actual vehicle state from Bullet
    local actUpVec = vehicle:getUpVector(ctx.scratchVector)
    local actUpX = HeliUtil.toLuaNum(actUpVec:x())
    local actUpY = HeliUtil.toLuaNum(actUpVec:y())
    local actUpZ = HeliUtil.toLuaNum(actUpVec:z())
    local actFwdVec = vehicle:getForwardVector(ctx.scratchVector)
    local actFwdX = HeliUtil.toLuaNum(actFwdVec:x())
    local actFwdY = HeliUtil.toLuaNum(actFwdVec:y())
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))

    -- Body right = up x forward
    local actRightX = actUpY * actFwdZ - actUpZ * actFwdY
    local actRightY = actUpZ * actFwdX - actUpX * actFwdZ
    local actRightZ = actUpX * actFwdY - actUpY * actFwdX

    ADRCOrientation.updateActualState(actUpX, actUpY, actUpZ, actFwdX, actFwdY, actFwdZ, actYawDeg)

    -- 2. Key input -> rotation deltas with first-order rate smoothing
    local heliType = ctx.heliType
    local blocked = ctx.blocked
    local fpsMultiplier = ctx.fpsMultiplier

    local basicAccelRate = HeliList[heliType].BasicAccelerationModifier or 0.15
    local maxSpeed = HeliList[heliType].MaxSpeed or 0.3
    if not HeliList[heliType].BasicAccelerationModifier then
        basicAccelRate = 0.4; maxSpeed = 0.15
    end
    local angle_90 = math.rad(90)

    -- Yaw (A/D)
    local rawYawRate = 0
    local isRotating = false
    if keys.a then rawYawRate = HeliConfig.GetYawRotationSpeed(); isRotating = true end
    if keys.d then rawYawRate = -HeliConfig.GetYawRotationSpeed(); isRotating = true end

    -- Pitch (UP/DOWN)
    local rawPitchRate = 0
    if keys.up and not keys.left and not keys.right then
        local bodyPitch = ADRCOrientation.getBodyPitch()
        if bodyPitch < angle_90 + maxSpeed and not blocked.up then
            rawPitchRate = basicAccelRate
        end
    elseif keys.down and not keys.left and not keys.right then
        local bodyPitch = ADRCOrientation.getBodyPitch()
        if bodyPitch > angle_90 - maxSpeed and not blocked.down then
            rawPitchRate = -basicAccelRate
        end
    end

    -- Roll (LEFT/RIGHT)
    local rawRollRate = 0
    if keys.left and not keys.up and not keys.down then
        local bodyRoll = ADRCOrientation.getBodyRoll()
        if bodyRoll < angle_90 + maxSpeed and not blocked.left then
            rawRollRate = -basicAccelRate
        end
    elseif keys.right and not keys.up and not keys.down then
        local bodyRoll = ADRCOrientation.getBodyRoll()
        if bodyRoll > angle_90 - maxSpeed and not blocked.right then
            rawRollRate = basicAccelRate
        end
    end

    -- First-order smoothing
    local tau = HeliConfig.GetAdrcInputSmoothingTau()
    local dt_input = 1.0 / ctx.fps
    if tau > 0.001 then
        local alpha = 1.0 - math.exp(-dt_input / tau)
        _smoothedYawRate   = _smoothedYawRate   + (rawYawRate   - _smoothedYawRate)   * alpha
        _smoothedPitchRate = _smoothedPitchRate + (rawPitchRate - _smoothedPitchRate) * alpha
        _smoothedRollRate  = _smoothedRollRate  + (rawRollRate  - _smoothedRollRate)  * alpha
    else
        _smoothedYawRate   = rawYawRate
        _smoothedPitchRate = rawPitchRate
        _smoothedRollRate  = rawRollRate
    end

    local pitchDelta = _smoothedPitchRate * fpsMultiplier
    local rollDelta  = _smoothedRollRate  * fpsMultiplier
    local yawDelta   = _smoothedYawRate   * fpsMultiplier

    -- 3. Apply tilt + yaw to orientation
    ADRCOrientation.applyTilt(pitchDelta, rollDelta)
    ADRCOrientation.applyYaw(yawDelta)

    -- Decay tilt to level when no directional input
    local hasTiltInput = keys.up or keys.down or keys.left or keys.right
    if not hasTiltInput then
        local decayPerSec = HeliConfig.GetAdrcTiltDecayRate()
        local dt_decay = 1.0 / ctx.fps
        ADRCOrientation.decayTiltToLevel(1.0 - math.exp(-decayPerSec * dt_decay))
    end

    -- 4. Yaw controller: direct damping during coast.
    -- While rotating: desired advances by input rate.
    -- On release: set desired = actual every frame (zero ADRC yaw error),
    -- and apply direct velocity-proportional yaw damping torque via couple force.
    -- This bypasses the yaw ADRC during coast — the ESO oscillates at high
    -- yaw rates because the discrete gains overcorrect. Direct damping is
    -- smooth and predictable: torque = -dampCoeff * yawRate * I_yaw.
    local YAW_DAMP_COEFF = 5.0   -- damping coefficient (higher = faster stop)
    local YAW_LOCK_RATE = 2.0    -- deg/s — lock heading when below this
    if not _desiredYawDeg then
        _desiredYawDeg = actYawDeg
    end
    -- Measure yaw rate
    local measuredYawRate = 0  -- deg/s
    if _prevActYawDeg then
        measuredYawRate = wrapAngle(actYawDeg - _prevActYawDeg) * ctx.fps
    end
    if isRotating then
        _desiredYawDeg = _desiredYawDeg + yawDelta
        if _desiredYawDeg > 180 then _desiredYawDeg = _desiredYawDeg - 360
        elseif _desiredYawDeg < -180 then _desiredYawDeg = _desiredYawDeg + 360
        end
        _yawCoasting = true
        _yawDampTorque = 0
    else
        if _yawCoasting then
            -- Coast: track actual (zero yaw error for ADRC) + direct damping
            _desiredYawDeg = actYawDeg
            if math.abs(measuredYawRate) < YAW_LOCK_RATE then
                -- Rotation stopped — lock heading, end coast
                _desiredYawDeg = actYawDeg
                _yawCoasting = false
                _yawDampTorque = 0
            else
                -- Apply direct yaw damping torque (will be added to couple forces)
                local Iy = ADRCTorqueController.getInertia()  -- returns Ix,Iy,Iz,valid
                local _, I_yaw_val = ADRCTorqueController.getInertia()
                _yawDampTorque = -YAW_DAMP_COEFF * math.rad(measuredYawRate) * I_yaw_val
            end
        else
            _yawDampTorque = 0
        end
        ADRCOrientation.setYaw(_desiredYawDeg)
    end
    _prevActYawDeg = actYawDeg

    -- 5. ADRC torque controller
    local desUpX, desUpY, desUpZ = ADRCOrientation.getDesiredUpVector()

    -- Desired yaw from full quaternion
    local desQuat = ADRCOrientation.getQuaternion()
    local desFwdX = 2 * (desQuat.x * desQuat.z + desQuat.w * desQuat.y)
    local desFwdZ = 1 - 2 * (desQuat.x * desQuat.x + desQuat.y * desQuat.y)
    local rawDesYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))

    -- Rate-limit desired yaw lead
    local maxYawLead = HeliConfig.GetAdrcMaxYawLead()
    local yawLead = wrapAngle(rawDesYawDeg - actYawDeg)
    local desYawDeg
    if yawLead > maxYawLead then
        desYawDeg = actYawDeg + maxYawLead
    elseif yawLead < -maxYawLead then
        desYawDeg = actYawDeg - maxYawLead
    else
        desYawDeg = rawDesYawDeg
    end

    -- ESO bandwidth ramp is handled internally by ADRCTorqueController
    -- (applies to both update and updateGround uniformly).
    local dt = 1.0 / ctx.fps
    local torqueX, torqueY, torqueZ, angErrMag,
          ctrlDesYaw, ctrlActYaw, ctrlErrY,
          esoRatePitch, esoRateYaw, esoRateRoll,
          Ix, Iz,
          esoDistPitch, esoDistRoll, esoDistYaw,
          esoErrPitch, esoErrRoll,
          woActual =
        ADRCTorqueController.compute(
            desUpX, desUpY, desUpZ, desYawDeg,
            actUpX, actUpY, actUpZ, actYawDeg,
            actRightX, actRightY, actRightZ,
            actFwdX, actFwdY, actFwdZ,
            dt, ctx.subSteps)

    -- Apply body-frame torque via body-aligned couple forces.
    -- compute() returns body-frame torque (pitch/yaw/roll around body axes).
    local substepMul = 1
    if HeliConfig.GetAdrcSubstepCompensation() >= 1 then
        substepMul = math.max(ctx.subSteps or 1, 1)
    end
    local appliedYawTorque = torqueY * substepMul
    if _yawCoasting and _yawDampTorque ~= 0 then
        appliedYawTorque = _yawDampTorque
    end
    ADRCCoupleForce.applyBodyAligned(vehicle,
        torqueX * substepMul, appliedYawTorque, torqueZ * substepMul,
        actRightX, actRightY, actRightZ,
        actUpX, actUpY, actUpZ,
        actFwdX, actFwdY, actFwdZ)

    -- 6. Velocity damping (drag model) -- replaces sim+correction pipeline
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ
    local hSpeed = VelocityUtil.horizontalSpeed(velX, velZ)
    local dragFX, dragFZ = 0, 0

    -- Drag always active: acts as air resistance limiting horizontal speed.
    -- Without continuous drag, body-up thrust at any tilt creates runaway
    -- horizontal acceleration (thrust → horizontal component → speed → no brake).
    -- When tilting intentionally, drag opposes the thrust's horizontal component,
    -- creating an equilibrium speed proportional to tilt angle — natural feel.
    -- When level with no input, drag brakes to a stop.
    if hSpeed > 0.01 then
        local dragCoeff = HeliConfig.GetAdrcDragCoeff()
        local dampForce = -dragCoeff * mass
        dragFX = velX * dampForce
        dragFZ = velZ * dampForce
        ctx.applyForce(dragFX, 0, dragFZ)
    end

    -- 7. Vertical control (same as framework pattern)
    local alphaV = ADRCEngine.VERTICAL_VELOCITY_SMOOTHING
    _smoothedVelY = alphaV * velY + (1.0 - alphaV) * _smoothedVelY

    local rawTargetVelY, gravComp, vBraking, engineDead = FlightModel.computeVerticalTarget(ctx, freeMode)

    -- Landing zone taper
    local currentAltitude = ctx.currentAltitude
    local groundLevelZ = ctx.groundLevelZ
    if rawTargetVelY < 0 and currentAltitude < groundLevelZ + HeliConfig.GetAdrcLandingZoneHeight() then
        local landingFactor = math.max((currentAltitude - groundLevelZ) / HeliConfig.GetAdrcLandingZoneHeight(), 0)
        landingFactor = math.max(landingFactor, HeliConfig.GetAdrcLandingMinSpeedFactor())
        rawTargetVelY = rawTargetVelY * landingFactor
    end

    -- Ramp vertical target to prevent force spikes
    local rampRate = 4.0
    local dt_ramp = 1.0 / ctx.fps
    _rampedTargetVelY = _rampedTargetVelY + (rawTargetVelY - _rampedTargetVelY) * (1.0 - math.exp(-rampRate * dt_ramp))
    local targetVelY = _rampedTargetVelY

    -- Adaptive vertical gain
    local absTarget = math.abs(targetVelY)
    if absTarget > ADRCEngine.ADAPTIVE_GAIN_DEADZONE then
        local absActual = math.abs(_smoothedVelY)
        if absActual > absTarget * 0.1 then
            local ratio = absActual / absTarget
            local desired = math.min(1.0 / ratio, ADRCEngine.ADAPTIVE_GAIN_MAX)
            desired = math.max(desired, ADRCEngine.ADAPTIVE_GAIN_MIN)
            local a = ADRCEngine.ADAPTIVE_GAIN_ALPHA
            _adaptiveGainMultiplier = a * desired + (1.0 - a) * _adaptiveGainMultiplier
        end
    end

    local verticalGain = HeliConfig.GetVerticalGain() * _adaptiveGainMultiplier
    local gravity = HeliConfig.GetGravity()
    local verticalForce = ForceComputer.computeThrustForce(
        targetVelY, _smoothedVelY, mass, verticalGain, gravity,
        ctx.subSteps, ctx.physicsDelta, gravComp)
    -- World-up thrust with adaptive gravity trim.
    -- Problem: our force applies in 1 of N substeps, gravity acts every substep.
    -- Instead of computing extra gravity from N (which jitters 3↔4 causing bias),
    -- learn the correct trim from observed drift. During hover (targetVelY≈0),
    -- any persistent velY drift means our gravity compensation is wrong.
    -- An integrator accumulates the error and adjusts the trim force until drift = 0.
    local nSteps = math.max(ctx.subSteps or 1, 1)
    local trimAlpha = 0.02  -- integrator speed (higher = faster convergence, more noise)
    local trimDecay = 0.005 -- leaky integrator: bleeds 0.5% per frame toward zero
    if math.abs(targetVelY) < 0.5 and gravComp then
        -- Hover/near-hover: integrate velocity error into trim
        _gravTrim = _gravTrim + _smoothedVelY * mass * trimAlpha
    end
    -- Always decay: prevents stale trim from transients
    _gravTrim = _gravTrim * (1 - trimDecay)
    -- Clamp to reasonable range
    local maxTrim = mass * gravity * 3
    if _gravTrim > maxTrim then _gravTrim = maxTrim end
    if _gravTrim < -maxTrim then _gravTrim = -maxTrim end
    -- Always apply: ForceComputer output + base substep compensation + learned trim
    local baseExtraGrav = 0
    if gravComp and nSteps > 1 then
        baseExtraGrav = mass * gravity * (nSteps - 1)
    end
    if verticalForce ~= 0 or baseExtraGrav ~= 0 or _gravTrim ~= 0 then
        ctx.applyForce(0, verticalForce + baseExtraGrav - _gravTrim, 0)
    end

    -- 8. Display speed from actual Bullet velocity (not sim)
    local displaySpeed = CoordUtil.msToKmh(hSpeed)

    -- Persist debug state
    local desAngleX, desAngleY, desAngleZ = ADRCOrientation.toEuler()
    _dbg.torqueX = safeNum(torqueX)
    _dbg.torqueY = safeNum(torqueY)
    _dbg.torqueZ = safeNum(torqueZ)
    _dbg.angErrMag = safeNum(angErrMag)
    _dbg.targetVelY = safeNum(targetVelY)
    _dbg.actUpX = safeNum(actUpX); _dbg.actUpY = safeNum(actUpY); _dbg.actUpZ = safeNum(actUpZ)
    _dbg.desUpX = safeNum(desUpX); _dbg.desUpY = safeNum(desUpY); _dbg.desUpZ = safeNum(desUpZ)
    _dbg.desAngleX = safeNum(desAngleX); _dbg.desAngleY = safeNum(desAngleY); _dbg.desAngleZ = safeNum(desAngleZ)
    _dbg.actAngleX = safeNum(ctx.angleX); _dbg.actAngleY = safeNum(ctx.angleY); _dbg.actAngleZ = safeNum(ctx.angleZ)
    _dbg.ctrlDesYaw = safeNum(ctrlDesYaw or 0)
    _dbg.ctrlActYaw = safeNum(ctrlActYaw or 0)
    _dbg.ctrlErrY = safeNum(ctrlErrY or 0)
    _dbg.esoRatePitch = safeNum(esoRatePitch or 0)
    _dbg.esoRateYaw = safeNum(esoRateYaw or 0)
    _dbg.esoRateRoll = safeNum(esoRateRoll or 0)
    _dbg.Ix = safeNum(Ix or 0); _dbg.Iz = safeNum(Iz or 0)
    _dbg.esoDistPitch = safeNum(esoDistPitch or 0); _dbg.esoDistRoll = safeNum(esoDistRoll or 0); _dbg.esoDistYaw = safeNum(esoDistYaw or 0)
    _dbg.esoErrPitch = safeNum(esoErrPitch or 0); _dbg.esoErrRoll = safeNum(esoErrRoll or 0)
    _dbg.woEffective = safeNum(woActual or 0)
    _dbg.mass = safeNum(mass)
    _dbg.vertForce = safeNum(verticalForce)
    _dbg.pitchDelta = safeNum(pitchDelta); _dbg.rollDelta = safeNum(rollDelta)
    _dbg.yawLead = safeNum(wrapAngle(rawDesYawDeg - actYawDeg))
    _dbg.dragFX = safeNum(dragFX); _dbg.dragFZ = safeNum(dragFZ)
    _dbg.hSpeed = safeNum(hSpeed); _dbg.displaySpeed = safeNum(displaySpeed)

    -- 9. Return results (no dualPathActive -- ADRC has no correction force path)
    return {
        engineDead = engineDead,
        dualPathActive = false,
        displaySpeed = displaySpeed,
        isBlockedHit = false,
        telemetrySpeed = displaySpeed,
        targetVelY = targetVelY,
        gravComp = gravComp,
        hasHInput = hasTiltInput,
        freeMode = freeMode,
        noHInput = not hasTiltInput,
        -- ADRC-specific debug in result
        torqueX = torqueX, torqueY = torqueY, torqueZ = torqueZ,
        angErrMag = angErrMag,
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: updateGround(ctx) -- ground mode with torque orientation hold
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFGroundResult
function ADRCEngine.updateGround(ctx)
    local vehicle = ctx.vehicle
    local keys = ctx.keys
    local mass = ctx.mass
    local velX, velY, velZ = ctx.velX, ctx.velY, ctx.velZ
    local heightAboveGround = ctx.currentAltitude - ctx.groundLevelZ

    local BOTTOM = HeliConfig.TRANSITION_ZONE_BOTTOM
    local TOP    = HeliConfig.TRANSITION_ZONE_TOP
    local t = math.max(0, math.min(1, (heightAboveGround - BOTTOM) / (TOP - BOTTOM)))

    local inTransition = (t > 0)
    local liftoff = false

    ADRCTorqueController.initFromVehicle(vehicle)

    if not ADRCOrientation.isInitialized() then
        ADRCOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- Read actual vehicle state from Bullet
    local actUpVec = vehicle:getUpVector(ctx.scratchVector)
    local actUpX = HeliUtil.toLuaNum(actUpVec:x())
    local actUpY = HeliUtil.toLuaNum(actUpVec:y())
    local actUpZ = HeliUtil.toLuaNum(actUpVec:z())
    local actFwdVec = vehicle:getForwardVector(ctx.scratchVector)
    local actFwdX = HeliUtil.toLuaNum(actFwdVec:x())
    local actFwdY = HeliUtil.toLuaNum(actFwdVec:y())
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))
    local actRightX = actUpY * actFwdZ - actUpZ * actFwdY
    local actRightY = actUpZ * actFwdX - actUpX * actFwdZ
    local actRightZ = actUpX * actFwdY - actUpY * actFwdX

    ADRCOrientation.updateActualState(actUpX, actUpY, actUpZ, actFwdX, actFwdY, actFwdZ, actYawDeg)

    -- Desired up from tilt only
    local desUpX, desUpY, desUpZ = ADRCOrientation.getDesiredUpVector()
    local desQuat = ADRCOrientation.getQuaternion()
    local desFwdX = 2 * (desQuat.x * desQuat.z + desQuat.w * desQuat.y)
    local desFwdZ = 1 - 2 * (desQuat.x * desQuat.x + desQuat.y * desQuat.y)
    local desYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))

    -- Only apply couple force torque in the transition zone (t > 0) or when
    -- ascending (W key). On the pure ground, couple forces are counterproductive:
    -- the ESO accumulates phantom disturbance from the substep mismatch and
    -- ground constraint (vehicle can't rotate freely), and maxTorque (200kNm)
    -- overwhelms gravity's restoring torque (5.6kNm) by 35×. When the ground
    -- constraint releases at liftoff, the accumulated bias flips the helicopter.
    -- Gravity + terrain collision hold orientation on the ground — no couple forces needed.
    local applyCoupleForces = inTransition or (keys.w and ctx.fuelPercent > 0)

    local dt = 1.0 / ctx.fps
    local torqueX, torqueY, torqueZ = ADRCTorqueController.compute(
        desUpX, desUpY, desUpZ, desYawDeg,
        actUpX, actUpY, actUpZ, actYawDeg,
        actRightX, actRightY, actRightZ,
        actFwdX, actFwdY, actFwdZ,
        dt, ctx.subSteps)

    if applyCoupleForces then
        local substepMul = 1
        if HeliConfig.GetAdrcSubstepCompensation() >= 1 then
            substepMul = math.max(ctx.subSteps or 1, 1)
        end
        ADRCCoupleForce.applyBodyAligned(vehicle,
            torqueX * substepMul, torqueY * substepMul, torqueZ * substepMul,
            actRightX, actRightY, actRightZ,
            actUpX, actUpY, actUpZ,
            actFwdX, actFwdY, actFwdZ)
    end

    -- Vertical forces (same pattern as framework ground mode)
    if keys.w and ctx.fuelPercent > 0 then
        ctx.setPhysicsActive(true)
        if ctx.subSteps > 0 then
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local ascendSpeed = HeliConfig.GetAscend()
            local thrustY = ForceComputer.computeThrustForce(
                ascendSpeed, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, true)
            local groundHold = (1.0 - t) * HeliConfig.GetAdrcGroundVelocityKill()
            ctx.applyForce(
                -velX * mass * groundHold,
                thrustY,
                -velZ * mass * groundHold)
        end
        liftoff = true

    elseif inTransition then
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GetAdrcGroundVelocityThreshold() then
            local killFactor = HeliConfig.GetAdrcGroundVelocityKill() * (1.0 - t)
            ctx.applyForce(
                -velX * mass * killFactor,
                0,
                -velZ * mass * killFactor)
        end
        if ctx.subSteps > 0 then
            local freeMode = vehicle:getModData().AutoBalance == true
            local targetVelY_g, gravComp_g = FlightModel.computeVerticalTarget(ctx, freeMode)
            if targetVelY_g < 0 and ctx.currentAltitude < ctx.groundLevelZ + HeliConfig.GetAdrcLandingZoneHeight() then
                local landFactor = math.max((ctx.currentAltitude - ctx.groundLevelZ) / HeliConfig.GetAdrcLandingZoneHeight(), 0)
                landFactor = math.max(landFactor, HeliConfig.GetAdrcLandingMinSpeedFactor())
                targetVelY_g = targetVelY_g * landFactor
            end
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local forceY = ForceComputer.computeThrustForce(
                targetVelY_g, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, gravComp_g)
            ctx.applyForce(0, forceY, 0)
        end

    else
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GetAdrcGroundVelocityThreshold() then
            local killF = HeliConfig.GetAdrcGroundVelocityKill()
            ctx.applyForce(
                -velX * mass * killF,
                -velY * mass * killF,
                -velZ * mass * killF)
        end
    end

    return {
        liftoff = liftoff,
        displaySpeed = 0,
        keepFlightState = true,
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: applyCorrectionForces (optional, ADRC does not use)
-- Not implemented -- ADRC returns dualPathActive=false, framework never calls this.
-------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------
-- IFlightEngine: Tunables
-------------------------------------------------------------------------------------

local TUNABLE_NAMES = {
    { name = "adrcEsoWo",           label = "ESO Wo" },
    { name = "adrcWcTilt",          label = "Tilt Wc" },
    { name = "adrcWcYaw",           label = "Yaw Wc" },
    { name = "adrcCoupleOffset",    label = "Couple Ofs" },
    { name = "adrcMaxTorque",       label = "Max Torque" },
    { name = "adrcInputSmoothingTau", label = "Smooth Tau" },
    { name = "adrcTiltDecayRate",   label = "Tilt Decay" },
    { name = "adrcDragCoeff",       label = "Drag Coeff" },
    { name = "adrcDragTiltThreshold", label = "Drag Tilt" },
    { name = "adrcGyroScale",       label = "Gyro Scale" },
}

function ADRCEngine.getTunables()
    local PARAMS = HeliConfig.getParamDefs()
    local result = {}
    for _, t in ipairs(TUNABLE_NAMES) do
        local p = PARAMS[t.name]
        result[#result + 1] = {
            name = t.name,
            label = t.label,
            value = HeliConfig.get(t.name),
            min = p and p.min or 0,
            max = p and p.max or 100,
            default = p and p.default or 0,
        }
    end
    return result
end

function ADRCEngine.getTunable(name)
    return HeliConfig.get(name)
end

function ADRCEngine.setTunable(name, value)
    HeliConfig.set(name, value)
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Sandbox Options
-------------------------------------------------------------------------------------

function ADRCEngine.getSandboxOptions()
    local PARAMS = HeliConfig.getParamDefs()
    local options = {}
    for _, p in pairs(PARAMS) do
        if p.field and p.ns == "ADRC" then
            options[#options + 1] = {
                field = p.field, type = "double",
                default = p.default, min = p.min, max = p.max, desc = p.desc,
            }
        end
    end
    return { namespace = "ADRC", options = options }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Debug
-------------------------------------------------------------------------------------

local DEBUG_COLUMNS = {
    -- Torque controller
    "torqueX", "torqueY", "torqueZ",
    "angErrMag",
    -- Desired vs actual angles
    "desAngleX", "desAngleY", "desAngleZ",
    "actAngleX", "actAngleY", "actAngleZ",
    -- Up-vectors
    "actUpX", "actUpY", "actUpZ",
    "desUpX", "desUpY", "desUpZ",
    -- Controller internals
    "ctrlDesYaw", "ctrlActYaw", "ctrlErrY",
    "esoRatePitch", "esoRateYaw", "esoRateRoll",
    "Ix", "Iz",
    "esoDistPitch", "esoDistRoll", "esoDistYaw",
    "esoErrPitch", "esoErrRoll",
    "woEffective", "mass",
    -- Forces
    "vertForce", "dragFX", "dragFZ",
    -- Input
    "pitchDelta", "rollDelta", "yawLead",
    -- Speed
    "hSpeed", "displaySpeed",
    "targetVelY",
}

function ADRCEngine.getDebugColumns()
    return DEBUG_COLUMNS
end

function ADRCEngine.getDebugState()
    local Ix, Iy, Iz, inertiaValid = ADRCTorqueController.getInertia()
    return {
        torqueX = _dbg.torqueX, torqueY = _dbg.torqueY, torqueZ = _dbg.torqueZ,
        angErrMag = _dbg.angErrMag,
        desAngleX = _dbg.desAngleX, desAngleY = _dbg.desAngleY, desAngleZ = _dbg.desAngleZ,
        actAngleX = _dbg.actAngleX, actAngleY = _dbg.actAngleY, actAngleZ = _dbg.actAngleZ,
        actUpX = _dbg.actUpX, actUpY = _dbg.actUpY, actUpZ = _dbg.actUpZ,
        desUpX = _dbg.desUpX, desUpY = _dbg.desUpY, desUpZ = _dbg.desUpZ,
        ctrlDesYaw = _dbg.ctrlDesYaw, ctrlActYaw = _dbg.ctrlActYaw, ctrlErrY = _dbg.ctrlErrY,
        esoRatePitch = _dbg.esoRatePitch, esoRateYaw = _dbg.esoRateYaw, esoRateRoll = _dbg.esoRateRoll,
        Ix = _dbg.Ix, Iz = _dbg.Iz,
        esoDistPitch = _dbg.esoDistPitch, esoDistRoll = _dbg.esoDistRoll, esoDistYaw = _dbg.esoDistYaw,
        esoErrPitch = _dbg.esoErrPitch, esoErrRoll = _dbg.esoErrRoll,
        woEffective = _dbg.woEffective, mass = _dbg.mass,
        vertForce = _dbg.vertForce, dragFX = _dbg.dragFX, dragFZ = _dbg.dragFZ,
        pitchDelta = _dbg.pitchDelta, rollDelta = _dbg.rollDelta, yawLead = _dbg.yawLead,
        hSpeed = _dbg.hSpeed, displaySpeed = _dbg.displaySpeed,
        targetVelY = _dbg.targetVelY,
        -- Extra (not in columns, available via commands)
        Iy = Iy, inertiaValid = inertiaValid,
    }
end

function ADRCEngine.getIntendedYaw()
    return _desiredYawDeg or 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Commands
-------------------------------------------------------------------------------------

function ADRCEngine.getCommands()
    return {
        { name = "inertia", args = "", description = "Show computed inertia tensor" },
    }
end

function ADRCEngine.executeCommand(name, argsString)
    if name == "inertia" then
        local Ix, Iy, Iz, valid = ADRCTorqueController.getInertia()
        if valid then
            return string.format("Inertia: Ix=%.1f Iy=%.1f Iz=%.1f (pitch/yaw/roll)", Ix, Iy, Iz)
        else
            return "Inertia not yet computed (enter vehicle first)"
        end
    end
    return "Unknown ADRC command: " .. tostring(name)
end

-------------------------------------------------------------------------------------
-- Register with framework
-------------------------------------------------------------------------------------
if IFlightEngine then
    IFlightEngine.register("ADRC", ADRCEngine)
else
    local function _deferredRegister()
        IFlightEngine.register("ADRC", ADRCEngine)
        Events.OnGameStart.Remove(_deferredRegister)
    end
    Events.OnGameStart.Add(_deferredRegister)
end
