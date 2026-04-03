--[[
    TRQEngine — Torque-based flight engine implementing IFlightEngine

    Torque-based flight engine: replaces setAngles teleport with couple-force
    torque so Bullet handles rotation natively (interpolated, no jitter).

    Fully self-contained: uses Core/Toolkit modules + TRQ-specific modules.
    Zero FBW dependencies. Each engine (FBW or TRQ) is independent.

    Registers itself at file scope: IFlightEngine.register("TRQ", TRQEngine)
]]

TRQEngine = {}

-- Constants (vertical velocity smoothing, adaptive gain)
TRQEngine.VERTICAL_VELOCITY_SMOOTHING = 0.3
TRQEngine.ADAPTIVE_GAIN_ALPHA   = 0.05
TRQEngine.ADAPTIVE_GAIN_MIN     = 1.0
TRQEngine.ADAPTIVE_GAIN_MAX     = 8.0
TRQEngine.ADAPTIVE_GAIN_DEADZONE = 0.3

-------------------------------------------------------------------------------------
-- Engine state
--- Wrap angle delta to [-180, +180].
local function wrapAngle(d)
    if d > 180 then d = d - 360
    elseif d < -180 then d = d + 360
    end
    return d
end

-------------------------------------------------------------------------------------
-- Force application at center of mass.
-- applyImpulseGeneric applies force at a WORLD POSITION. Bullet computes torque
-- from (applicationPoint - centerOfMass). If force is applied at model origin
-- (getX/Y/Z) but the CoM is offset, every force creates unintended torque.
-- UH-1B has CoM 1.6m forward of model origin: a 40kN vertical braking force
-- at model origin creates 64kNm of phantom pitch torque.
-- This function applies force at the CoM position to eliminate phantom torque.
-------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _tireInflationSet = false
-- _wasGroundMode removed: TRQ uses torque in ground mode (no setAngles discontinuity)
local _smoothedVelY = 0
-- Input rate smoothing: ramp key input rates through first-order filter
-- to prevent step changes that cause ADRC torque saturation.
local _smoothedYawRate = 0
local _smoothedPitchRate = 0
local _smoothedRollRate = 0
local _adaptiveGainMultiplier = 1.0
local _rampedTargetVelY = 0  -- smoothed vertical target (prevents 220kN force spikes)

-- Debug state packed into one table to stay under Kahlua's 60-upvalue limit.
-- All _last* fields are write-once-per-frame, read by getDebugState/recorder.
local _dbg = {
    torqueX=0, torqueY=0, torqueZ=0,
    omegaX=0, omegaY=0, omegaZ=0, angErrMag=0,
    desiredHX=0, desiredHZ=0, targetVelY=0,
    angleZ=0, angleX=0, fwdX=0, fwdZ=0,
    bodyTorqueX=0, bodyTorqueY=0, bodyTorqueZ=0,
    ctrlDesYaw=0, ctrlActYaw=0, ctrlErrY=0,
    ctrlWOmX=0, ctrlWOmY=0, ctrlWOmZ=0,
    effIwx=0, effIwz=0,
    corrFX=0, corrFZ=0, vertForce=0,
    pitchDelta=0, rollDelta=0, yawLead=0,
    rateCmdX=0, rateCmdZ=0, integralX=0, integralZ=0,
    actUpX=0, actUpY=0, actUpZ=0,
    desUpX=0, desUpY=0, desUpZ=0,
    desAngleX=0, desAngleY=0, desAngleZ=0,
    actAngleX=0, actAngleY=0, actAngleZ=0,
}

-------------------------------------------------------------------------------------
-- Toolkit instances
-------------------------------------------------------------------------------------
local _sim = SimModel2D.new(HeliConfig.TARGET_FPS)
local _errorTracker = ErrorTracker2D.new(HeliConfig.HISTORY_SIZE, 5)

-------------------------------------------------------------------------------------
-- IFlightEngine: Metadata
-------------------------------------------------------------------------------------

function TRQEngine.getInfo()
    return { name = "TRQ", version = "0.1", description = "Torque-based rotation (research prototype)" }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Lifecycle
-------------------------------------------------------------------------------------

function TRQEngine.resetFlightState()
    _hasTiltInput = false
    _hasHorizontalInput = false
    _flightAssistOff = false
    _warmupCounter = HeliConfig.GetTrqWarmupFrames()
    _simInitialized = false
    _smoothedVelY = 0
    _smoothedYawRate = 0
    _smoothedPitchRate = 0
    _smoothedRollRate = 0
    _rampedTargetVelY = 0
    _adaptiveGainMultiplier = 1.0

    _dbg.torqueX = 0
    _dbg.torqueY = 0
    _dbg.torqueZ = 0
    _dbg.bodyTorqueX = 0
    _dbg.bodyTorqueY = 0
    _dbg.bodyTorqueZ = 0
    _dbg.ctrlDesYaw = 0
    _dbg.ctrlActYaw = 0
    _dbg.ctrlErrY = 0
    _dbg.ctrlWOmX = 0
    _dbg.ctrlWOmY = 0
    _dbg.ctrlWOmZ = 0
    _dbg.effIwx = 0
    _dbg.effIwz = 0
    _dbg.corrFX = 0; _dbg.corrFZ = 0
    _dbg.vertForce = 0
    _dbg.pitchDelta = 0; _dbg.rollDelta = 0
    _dbg.yawLead = 0
    _dbg.rateCmdX = 0; _dbg.rateCmdZ = 0
    _dbg.integralX = 0; _dbg.integralZ = 0
    _dbg.omegaX = 0
    _dbg.omegaY = 0
    _dbg.omegaZ = 0
    _dbg.angErrMag = 0
    _dbg.desiredHX = 0
    _dbg.desiredHZ = 0
    _dbg.targetVelY = 0
    _dbg.angleZ = 0
    _dbg.angleX = 0
    _dbg.fwdX = 0
    _dbg.fwdZ = 0
    _dbg.actUpX = 0
    _dbg.actUpY = 0
    _dbg.actUpZ = 0
    _dbg.desUpX = 0
    _dbg.desUpY = 0
    _dbg.desUpZ = 0
    _dbg.desAngleX = 0
    _dbg.desAngleY = 0
    _dbg.desAngleZ = 0
    _dbg.actAngleX = 0
    _dbg.actAngleY = 0
    _dbg.actAngleZ = 0

    TRQOrientation.reset()
    TRQYawController.reset()
    TRQTiltResolver.reset()
    TRQAngularEstimator.reset()
    TRQTorqueController.reset()
    _sim:reset(0, 0)
    _errorTracker:reset()
    HeliForceAdapter.resetPhysicsTime()
    HeliVelocityAdapter.resetSmoothing()
end

local function _reinitSim(posX, posZ)
    _sim:reset(posX, posZ)
    _simInitialized = true
end

function TRQEngine.initFlight(vehicle)
    local posX = vehicle:getX()
    local posZ = vehicle:getY()
    if posX == nil or posZ == nil then return end
    _reinitSim(HeliUtil.toLuaNum(posX), HeliUtil.toLuaNum(posZ))

    -- Compute inertia tensor from vehicle extents (once per vehicle)
    TRQTorqueController.initFromVehicle(vehicle)

    -- Tire inflation will be set on first update() frame (deferred from here
    -- because Bullet wheel may not exist yet at initFlight time).
    _tireInflationSet = false
end

function TRQEngine.tickWarmup()
    if _warmupCounter > 0 then
        _warmupCounter = _warmupCounter - 1
    end
end

function TRQEngine.isWarmedUp()
    return _warmupCounter <= 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: update(ctx) — one frame of airborne flight
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFUpdateResult
function TRQEngine.update(ctx)
    local vehicle = ctx.vehicle

    -- No ground→airborne omega reset needed: TRQ uses torque in ground mode
    -- (same PD, no setAngles), so there's no velocity discontinuity at transition.

    -- Ensure inertia is computed (handles mid-flight engine switch where
    -- initFlight was never called because HeliMove skips warmup phase)
    TRQTorqueController.initFromVehicle(vehicle)

    local keys = ctx.keys
    local fpsMultiplier = ctx.fpsMultiplier
    local heliType = ctx.heliType
    local currentAltitude = ctx.currentAltitude
    local groundLevelZ = ctx.groundLevelZ
    local blocked = ctx.blocked

    local freeMode = vehicle:getModData().AutoBalance == true
    _flightAssistOff = freeMode

    -- 0. Set phantom wheel tire inflation (once, deferred from initFlight)
    -- Must happen after Bullet has fully created the vehicle + wheel.
    -- controlVehicle() reads tire inflation each frame; uninitialized (0.0)
    -- causes a braking penalty on the phantom wheel.
    if not _tireInflationSet then
        -- Retry silently — wheel index 0 may not exist on the first frame
        -- after engine switch. Succeeds once Bullet has fully created the wheel.
        local ok = pcall(vehicle.setTireInflation, vehicle, 0, 1.0)
        if ok then
            _tireInflationSet = true
        end
    end

    -- 1. Init TRQOrientation from vehicle if not initialized
    if not TRQOrientation.isInitialized() then
        TRQOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- 1b. Update actual vehicle state in TRQOrientation (read from Bullet)
    --     This must happen BEFORE InputProcessor reads body angles.
    local actUpVec = vehicle:getUpVector(ctx.scratchVector)
    local actUpX = HeliUtil.toLuaNum(actUpVec:x())
    local actUpY = HeliUtil.toLuaNum(actUpVec:y())
    local actUpZ = HeliUtil.toLuaNum(actUpVec:z())
    local actFwdVec = vehicle:getForwardVector(ctx.scratchVector)
    local actFwdX = HeliUtil.toLuaNum(actFwdVec:x())
    local actFwdY = HeliUtil.toLuaNum(actFwdVec:y())
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))
    -- Body right = up x forward (right-handed: Y x Z = X in Bullet coords)
    local actRightX = actUpY * actFwdZ - actUpZ * actFwdY
    local actRightY = actUpZ * actFwdX - actUpX * actFwdZ
    local actRightZ = actUpX * actFwdY - actUpY * actFwdX
    TRQOrientation.updateActualState(actUpX, actUpY, actUpZ, actFwdX, actFwdZ, actYawDeg)

    -- 2. Key input → rotation deltas (torque-tailored: NO auto-level in desired state)
    --    Auto-leveling is inherent in the PD: desired=level → error=tilt → corrective torque.
    --    InputProcessor's auto-level would create double-correction (desired moves AND PD corrects).
    --    Body angle limits still use ACTUAL tilt (from Bullet) to cap input range.
    local pitchDelta, yawDelta, rollDelta = 0, 0, 0
    local isRotating = false

    local basicAccelRate = HeliList[heliType].BasicAccelerationModifier or 0.15
    local maxSpeed = HeliList[heliType].MaxSpeed or 0.3
    if not HeliList[heliType].BasicAccelerationModifier then
        basicAccelRate = 0.4; maxSpeed = 0.15
    end
    local angle_90 = math.rad(90)

    -- Yaw (A/D) — raw target rate from key state
    local rawYawRate = 0
    if keys.a then rawYawRate = HeliConfig.GetYawRotationSpeed(); isRotating = true end
    if keys.d then rawYawRate = -HeliConfig.GetYawRotationSpeed(); isRotating = true end

    -- Pitch (UP/DOWN) — raw target rate from key state
    local rawPitchRate = 0
    if keys.up and not keys.left and not keys.right then
        local bodyPitch = TRQOrientation.getBodyPitch()
        if bodyPitch < angle_90 + maxSpeed and not blocked.up then
            rawPitchRate = basicAccelRate
        end
    elseif keys.down and not keys.left and not keys.right then
        local bodyPitch = TRQOrientation.getBodyPitch()
        if bodyPitch > angle_90 - maxSpeed and not blocked.down then
            rawPitchRate = -basicAccelRate
        end
    end

    -- Roll (LEFT/RIGHT) — raw target rate from key state
    local rawRollRate = 0
    if keys.left and not keys.up and not keys.down then
        local bodyRoll = TRQOrientation.getBodyRoll()
        if bodyRoll < angle_90 + maxSpeed and not blocked.left then
            rawRollRate = -basicAccelRate
        end
    elseif keys.right and not keys.up and not keys.down then
        local bodyRoll = TRQOrientation.getBodyRoll()
        if bodyRoll > angle_90 - maxSpeed and not blocked.right then
            rawRollRate = basicAccelRate
        end
    end

    -- First-order smoothing: ramp rates instead of stepping.
    -- Prevents ADRC torque saturation on key press/release.
    local tau = HeliConfig.GetTrqInputSmoothingTau()
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

    pitchDelta = _smoothedPitchRate * fpsMultiplier
    rollDelta  = _smoothedRollRate  * fpsMultiplier
    yawDelta   = _smoothedYawRate   * fpsMultiplier

    -- 3. Apply tilt + yaw to TRQOrientation (desired orientation)
    TRQOrientation.applyTilt(pitchDelta, rollDelta)
    TRQOrientation.applyYaw(yawDelta)

    -- When no directional keys pressed, gently decay desired tilt back to level.
    -- Without this, any tilt from key input persists forever after release.
    -- Rate 3.0/s: at 60fps → 0.05/frame, at 30fps → 0.10/frame. Same real-time speed.
    local hasTiltInput = keys.up or keys.down or keys.left or keys.right
    if not hasTiltInput then
        local decayPerSec = 3.0
        local dt_decay = 1.0 / ctx.fps
        TRQOrientation.decayTiltToLevel(1.0 - math.exp(-decayPerSec * dt_decay))
    end

    -- 4. Yaw MPC: track intended heading, hard lock when not rotating
    local simYaw = TRQYawController.update(TRQOrientation.getYaw(), isRotating, yawDelta)
    if not isRotating then
        TRQOrientation.setYaw(simYaw)
    end

    -- 5. TRQ: apply torque instead of setAngles
    --    Desired quaternion direct from TRQOrientation (no Euler round-trip).
    --    Desired yaw scalar from TRQOrientation (for scalar yaw PD).
    --    TRQTorqueController: tilt-decomposed pitch/roll + scalar yaw → torque.
    --
    -- Actual vehicle state already read in step 1b (actUpX/Y/Z, actFwdX/Z, actYawDeg)
    -- Reuse those values here — no duplicate Bullet reads.

    -- Desired up-vector from TILT ONLY (no heading component).
    -- Using the full quaternion (yawQ * tiltQ) would include heading in the up-vector,
    -- causing heading lag during yaw to appear as phantom tilt error.
    local desUpX, desUpY, desUpZ = TRQOrientation.getDesiredUpVector()

    -- Desired yaw from the full quaternion (heading IS needed for yaw PD).
    local desQuat = TRQOrientation.getQuaternion()
    local desFwdX = 2 * (desQuat.x * desQuat.z + desQuat.w * desQuat.y)
    local desFwdZ = 1 - 2 * (desQuat.x * desQuat.x + desQuat.y * desQuat.y)
    local rawDesYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))

    -- Rate-limit desired yaw: don't let desired race ahead of actual by more
    -- than MAX_YAW_LEAD degrees. FBW's yaw system advances desired at ~42°/s
    -- (constant rate from key input) but with torque the actual must accelerate
    -- through inertia. If desired leads by >180°, wrapAngle flips direction and
    -- the PD reverses → yaw explodes. Clamping prevents this.
    local MAX_YAW_LEAD = 45
    local yawLead = wrapAngle(rawDesYawDeg - actYawDeg)
    local desYawDeg
    if yawLead > MAX_YAW_LEAD then
        desYawDeg = actYawDeg + MAX_YAW_LEAD
    elseif yawLead < -MAX_YAW_LEAD then
        desYawDeg = actYawDeg - MAX_YAW_LEAD
    else
        desYawDeg = rawDesYawDeg
    end

    -- Omega: Euler angles still needed for quaternion-based estimator
    -- (it builds quaternion from Euler — unique, no flip issue)
    local dt = 1.0 / ctx.fps
    local omegaX, omegaY, omegaZ = TRQAngularEstimator.update(ctx.angleX, ctx.angleY, ctx.angleZ, dt)
    -- Body-frame ADRC: pass body axes so error is projected onto body pitch/roll.
    -- Returns body-frame torques: X=pitch, Y=yaw, Z=roll.
    local torqueX, torqueY, torqueZ, angErrMag,
          ctrlDesYaw, ctrlActYaw, ctrlErrY, ctrlWOmX, ctrlWOmY, ctrlWOmZ,
          effIwx, effIwz,
          rateCmdX, rateCmdZ, integralX, integralZ =
        TRQTorqueController.compute(
            desUpX, desUpY, desUpZ, desYawDeg,
            actUpX, actUpY, actUpZ, actYawDeg,
            actRightX, actRightY, actRightZ,
            actFwdX, actFwdY, actFwdZ,
            dt, ctx.subSteps)

    -- Apply body-frame torque via body-aligned couple forces.
    -- applyBodyAligned uses body axes to orient the couple force offsets/directions
    -- so the torque acts around body pitch/roll/yaw axes regardless of heading.
    --
    -- Substep compensation (A/B testable via trqSubstepCompensation param):
    -- Forces are drained in the first substep only (queue clears). When enabled,
    -- multiply by N so one substep delivers the full frame's angular impulse.
    -- When disabled (default), the ESO's x3 absorbs the 1/N mismatch.
    local substepMul = 1
    if HeliConfig.GetTrqSubstepCompensation() >= 1 then
        substepMul = math.max(ctx.subSteps or 1, 1)
    end
    TRQCoupleForce.applyBodyAligned(vehicle,
        torqueX * substepMul, torqueY * substepMul, torqueZ * substepMul,
        actRightX, actRightY, actRightZ,
        actUpX, actUpY, actUpZ,
        actFwdX, actFwdY, actFwdZ)

    -- Persist for debug
    local desAngleX, desAngleY, desAngleZ = TRQOrientation.toEuler()
    _dbg.torqueX = torqueX  -- world-frame (= actual applied torque)
    _dbg.torqueY = torqueY
    _dbg.torqueZ = torqueZ
    _dbg.bodyTorqueX = torqueX  -- same as world (uniform inertia, no body transform)
    _dbg.bodyTorqueY = torqueY
    _dbg.bodyTorqueZ = torqueZ
    _dbg.omegaX = omegaX
    _dbg.omegaY = omegaY
    _dbg.omegaZ = omegaZ
    _dbg.angErrMag = angErrMag
    _dbg.ctrlDesYaw = ctrlDesYaw or 0
    _dbg.ctrlActYaw = ctrlActYaw or 0
    _dbg.ctrlErrY = ctrlErrY or 0
    _dbg.ctrlWOmX = ctrlWOmX or 0
    _dbg.ctrlWOmY = ctrlWOmY or 0
    _dbg.ctrlWOmZ = ctrlWOmZ or 0
    _dbg.effIwx = effIwx or 0
    _dbg.effIwz = effIwz or 0
    _dbg.pitchDelta = pitchDelta
    _dbg.rollDelta = rollDelta
    _dbg.rateCmdX = rateCmdX or 0
    _dbg.rateCmdZ = rateCmdZ or 0
    _dbg.integralX = integralX or 0
    _dbg.integralZ = integralZ or 0
    _dbg.yawLead = wrapAngle(rawDesYawDeg - actYawDeg)
    _dbg.actUpX = actUpX
    _dbg.actUpY = actUpY
    _dbg.actUpZ = actUpZ
    _dbg.desUpX = desUpX  -- tilt-only up-vector (heading-independent)
    _dbg.desUpY = desUpY
    _dbg.desUpZ = desUpZ
    _dbg.actAngleX = ctx.angleX  -- raw Euler (for CSV readability)
    _dbg.actAngleY = ctx.angleY
    _dbg.actAngleZ = ctx.angleZ
    _dbg.desAngleX = desAngleX
    _dbg.desAngleY = desAngleY
    _dbg.desAngleZ = desAngleZ

    -- 6. Read forward direction + body angles from desired orientation
    -- Read body angles from TRQOrientation for the flight model
    local fwdX, fwdZ = TRQOrientation.getForward()
    local angleZ = TRQOrientation.getBodyPitch()
    local angleX = TRQOrientation.getBodyRoll()
    _dbg.angleZ = angleZ
    _dbg.angleX = angleX
    _dbg.fwdX = fwdX
    _dbg.fwdZ = fwdZ

    -- 7-8. Wall pre-blocking + FlightFilters pipeline → desired horizontal velocity
    local posX = ctx.posX
    local posZ = ctx.posZ
    local totalVelX, totalVelZ, totalSpeed, totalTiltRad, isBlockedHit =
        TRQTiltResolver.resolve(angleZ, angleX, blocked, fwdX, fwdZ, posX, posZ)

    -- 9. Tilt/input flags
    local noiseFloor = HeliConfig.GetTrqTiltNoiseFloor()
    local noInput = (totalTiltRad < noiseFloor * 2) or (totalSpeed < HeliConfig.GetTrqNoInputSpeedThreshold())
    local hasHInput = not noInput
    _hasTiltInput = hasHInput
    _hasHorizontalInput = hasHInput

    -- 10. FA-off coast logic → resolve desired velocity for sim
    local desiredHX, desiredHZ
    desiredHX, desiredHZ, hasHInput = SimController.resolveDesiredVelocity(
        hasHInput, totalVelX, totalVelZ, freeMode, noInput,
        _sim, ctx.velX, ctx.velZ, _reinitSim, posX, posZ)
    if hasHInput and not _hasHorizontalInput then
        _hasHorizontalInput = true
    end

    -- 11. Select effective inertia
    if not _simInitialized then
        _reinitSim(posX, posZ)
    end

    local fps = ctx.fps
    local deltaTime = 1.0 / fps
    local baseBrake = HeliConfig.GetBrake()
    local trqSimFactor = HeliConfig.GetTrqSimInertiaFactor()
    local effectiveInertia = baseBrake * (hasHInput and HeliConfig.GetAccel() or HeliConfig.GetDecel()) * trqSimFactor

    -- 12-14. Sim advance + heading reanchor + soft anchor
    SimController.advanceAndAnchor(_sim, _errorTracker, desiredHX, desiredHZ,
        deltaTime, effectiveInertia, hasHInput, posX, posZ, fps, _flightAssistOff, ctx.positionDeltaSpeed,
        TRQYawController.checkHeadingReanchor)

    -- 15. Record in error tracker
    local simPosX, simPosZ, simVelX, simVelZ = _sim:getState()
    _errorTracker:record(posX, posZ, simPosX, simPosZ)

    -- 16. Velocity from framework ctx
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ

    -- 16b. Smooth velY
    local alpha = TRQEngine.VERTICAL_VELOCITY_SMOOTHING
    _smoothedVelY = alpha * velY + (1.0 - alpha) * _smoothedVelY

    -- 17. Vertical target
    local rawTargetVelY, gravComp, vBraking, engineDead = FlightModel.computeVerticalTarget(ctx, freeMode)

    -- Landing zone taper
    if rawTargetVelY < 0 and currentAltitude < groundLevelZ + HeliConfig.GetTrqLandingZoneHeight() then
        local landingFactor = math.max((currentAltitude - groundLevelZ) / HeliConfig.GetTrqLandingZoneHeight(), 0)
        landingFactor = math.max(landingFactor, HeliConfig.GetTrqLandingMinSpeedFactor())
        rawTargetVelY = rawTargetVelY * landingFactor
    end

    -- Ramp the vertical target instead of stepping it instantly.
    -- FBW teleports orientation so a 220kN force spike from targetVelY jumping 0→-10
    -- doesn't cause tilt. TRQ's physics body gets hit by this spike, and any asymmetry
    -- in Bullet's response shows up as phantom tilt. Ramping over ~0.5s limits the peak
    -- force to ~22kN — gentle enough for the torque PD to maintain orientation.
    local rampRate = 4.0  -- reaches ~98% of target in 1 second (1 - e^(-4*1))
    local dt_ramp = 1.0 / ctx.fps
    _rampedTargetVelY = _rampedTargetVelY + (rawTargetVelY - _rampedTargetVelY) * (1.0 - math.exp(-rampRate * dt_ramp))
    local targetVelY = _rampedTargetVelY

    -- Persist flight model outputs for debug
    _dbg.desiredHX = desiredHX
    _dbg.desiredHZ = desiredHZ
    _dbg.targetVelY = targetVelY

    -- 18. Dual-path activation
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local actualHorizontalSpeed = VelocityUtil.horizontalSpeed(velX, velZ)
    local dualPathActive = TRQEngine.isWarmedUp() and
        (hasHInput or errMag > HeliConfig.GetTrqDualPathErrorThreshold() or actualHorizontalSpeed > HeliConfig.GetTrqDualPathSpeedThreshold())

    -- 19. Adaptive gain
    local absTarget = math.abs(targetVelY)
    if absTarget > TRQEngine.ADAPTIVE_GAIN_DEADZONE then
        local absActual = math.abs(_smoothedVelY)
        if absActual > absTarget * 0.1 then
            local ratio = absActual / absTarget
            local desired = math.min(1.0 / ratio, TRQEngine.ADAPTIVE_GAIN_MAX)
            desired = math.max(desired, TRQEngine.ADAPTIVE_GAIN_MIN)
            local a = TRQEngine.ADAPTIVE_GAIN_ALPHA
            _adaptiveGainMultiplier = a * desired + (1.0 - a) * _adaptiveGainMultiplier
        end
    end

    -- 19b. Vertical thrust
    local verticalGain = HeliConfig.GetVerticalGain() * _adaptiveGainMultiplier
    local gravity = HeliConfig.GetGravity()
    local verticalForce = ForceComputer.computeThrustForce(
        targetVelY, _smoothedVelY, ctx.mass, verticalGain, gravity,
        ctx.subSteps, ctx.physicsDelta, gravComp)
    _dbg.vertForce = verticalForce
    if verticalForce ~= 0 then
        ctx.applyForce(0, verticalForce, 0)
    end

    -- 20. Display speed
    local displaySpeed = CoordUtil.msToKmh(VelocityUtil.horizontalSpeed(simVelX, simVelZ))

    -- 21. Return results
    return {
        engineDead = engineDead,
        dualPathActive = dualPathActive,
        displaySpeed = displaySpeed,
        isBlockedHit = isBlockedHit,
        telemetrySpeed = displaySpeed,
        angleZ = angleZ, angleX = angleX,
        fwdX = fwdX, fwdZ = fwdZ,
        desiredVelX = desiredHX, desiredVelZ = desiredHZ,
        simVelX = simVelX, simVelZ = simVelZ,
        simPosX = simPosX, simPosZ = simPosZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
        targetVelY = targetVelY,
        gravComp = gravComp,
        hasHInput = hasHInput,
        freeMode = freeMode,
        noHInput = noInput,
        -- TRQ-specific debug fields
        torqueX = torqueX, torqueY = torqueY, torqueZ = torqueZ,
        omegaX = omegaX, omegaY = omegaY, omegaZ = omegaZ,
        angErrMag = angErrMag,
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: updateGround(ctx)
-- TRQ ground mode: uses TORQUE for orientation hold (never setAngles).
-- setAngles teleports the quaternion but does NOT reset angular velocity
-- (verified: btRigidBody::setCenterOfMassTransform at line 399-412 of
-- btRigidBody.cpp only sets m_worldTransform, leaves m_angularVelocity
-- untouched). Each frame of setAngles vs suspension accumulates angular
-- velocity that persists into airborne mode, causing immediate divergence.
-- With torque: no teleport, no velocity accumulation, smooth transition.
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFGroundResult
function TRQEngine.updateGround(ctx)
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

    -- Orientation hold via torque (same PD as airborne, no setAngles).
    -- Ensures zero accumulated angular velocity at ground→airborne transition.
    TRQTorqueController.initFromVehicle(vehicle)

    if not TRQOrientation.isInitialized() then
        TRQOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- Read actual vehicle state from Bullet and update TRQOrientation
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
    TRQOrientation.updateActualState(actUpX, actUpY, actUpZ, actFwdX, actFwdZ, actYawDeg)

    -- Desired up-vector from tilt only (heading-independent)
    local desUpX, desUpY, desUpZ = TRQOrientation.getDesiredUpVector()
    -- Desired yaw from full quaternion
    local desQuat = TRQOrientation.getQuaternion()
    local desFwdX = 2 * (desQuat.x * desQuat.z + desQuat.w * desQuat.y)
    local desFwdZ = 1 - 2 * (desQuat.x * desQuat.x + desQuat.y * desQuat.y)
    local desYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))

    local dt = 1.0 / ctx.fps
    local omegaX, omegaY, omegaZ = TRQAngularEstimator.update(ctx.angleX, ctx.angleY, ctx.angleZ, dt)
    local torqueX, torqueY, torqueZ = TRQTorqueController.compute(
        desUpX, desUpY, desUpZ, desYawDeg,
        actUpX, actUpY, actUpZ, actYawDeg,
        actRightX, actRightY, actRightZ,
        actFwdX, actFwdY, actFwdZ,
        dt, ctx.subSteps)

    -- Substep compensation: same toggle as main flight path
    local substepMul = 1
    if HeliConfig.GetTrqSubstepCompensation() >= 1 then
        substepMul = math.max(ctx.subSteps or 1, 1)
    end
    TRQCoupleForce.applyBodyAligned(vehicle,
        torqueX * substepMul, torqueY * substepMul, torqueZ * substepMul,
        actRightX, actRightY, actRightZ,
        actUpX, actUpY, actUpZ,
        actFwdX, actFwdY, actFwdZ)

    -- Sim re-anchor during transition
    if inTransition then
        _reinitSim(ctx.posX, ctx.posZ)
    end

    -- Vertical forces (same as FBW ground mode)
    if keys.w and ctx.fuelPercent > 0 then
        ctx.setPhysicsActive(true)
        if ctx.subSteps > 0 then
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local ascendSpeed = HeliConfig.GetAscend()
            local thrustY = ForceComputer.computeThrustForce(
                ascendSpeed, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, true)
            local groundHold = (1.0 - t) * HeliConfig.GetTrqGroundVelocityKill()
            ctx.applyForce(
                -velX * mass * groundHold,
                thrustY,
                -velZ * mass * groundHold)
        end
        liftoff = true

    elseif inTransition then
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GetTrqGroundVelocityThreshold() then
            local killFactor = HeliConfig.GetTrqGroundVelocityKill() * (1.0 - t)
            ctx.applyForce(
                -velX * mass * killFactor,
                0,
                -velZ * mass * killFactor)
        end
        if ctx.subSteps > 0 then
            local freeMode = ctx.vehicle:getModData().AutoBalance == true
            local targetVelY, gravComp = FlightModel.computeVerticalTarget(ctx, freeMode)
            if targetVelY < 0 and ctx.currentAltitude < ctx.groundLevelZ + HeliConfig.GetTrqLandingZoneHeight() then
                local landingFactor = math.max((ctx.currentAltitude - ctx.groundLevelZ) / HeliConfig.GetTrqLandingZoneHeight(), 0)
                landingFactor = math.max(landingFactor, HeliConfig.GetTrqLandingMinSpeedFactor())
                targetVelY = targetVelY * landingFactor
            end
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local forceY = ForceComputer.computeThrustForce(
                targetVelY, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, gravComp)
            ctx.applyForce(0, forceY, 0)
        end

    else
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GetTrqGroundVelocityThreshold() then
            ctx.applyForce(
                -velX * mass * HeliConfig.GetTrqGroundVelocityKill(),
                -velY * mass * HeliConfig.GetTrqGroundVelocityKill(),
                -velZ * mass * HeliConfig.GetTrqGroundVelocityKill())
        end
    end

    return {
        liftoff = liftoff,
        displaySpeed = 0,
        keepFlightState = true,  -- always keep state (torque PD runs continuously)
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: applyCorrectionForces — 0-frame delay path (identical to FBW)
-------------------------------------------------------------------------------------
--- @param cctx HEFCorrectionCtx
function TRQEngine.applyCorrectionForces(cctx)
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local errMag = math.sqrt(errX * errX + errZ * errZ)

    local fx, fz = ForceComputer.computeCorrectionForce(
        errX, errZ, errRateX, errRateZ, errMag,
        HeliConfig.GetPositionProportionalGain(), HeliConfig.GetPositionDerivativeGain(),
        cctx.velX, cctx.velZ, cctx.mass, HeliConfig.GetTrqVelForceFactor(),
        HeliConfig.GetFinalStopDampingGain(), _flightAssistOff,
        HeliConfig.GetTrqFaOffDeadzone(), HeliConfig.GetTrqFaOffMinDamping())

    _dbg.corrFX = fx
    _dbg.corrFZ = fz
    cctx.applyForce(fx, 0, fz)
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Tunables
-------------------------------------------------------------------------------------

local TUNABLE_NAMES = {
    -- TRQ angular PD
    { name = "trqPitchPGain",   label = "Pitch P" },
    { name = "trqPitchDGain",   label = "Pitch D" },
    { name = "trqRollPGain",    label = "Roll P" },
    { name = "trqRollDGain",    label = "Roll D" },
    { name = "trqYawPGain",     label = "Yaw P" },
    { name = "trqYawDGain",     label = "Yaw D" },
    { name = "trqCoupleOffset", label = "Couple Ofs" },
    { name = "trqOmegaAlpha",   label = "Omega EMA" },
    { name = "trqMaxTorque",    label = "Max Torque" },
    -- Shared horizontal PD (framework params)
    { name = "positionProportionalGain", label = "Pos P" },
    { name = "positionDerivativeGain",   label = "Pos D" },
    { name = "maxPositionError",         label = "Max Error" },
    { name = "finalStopDampingGain",     label = "Stop Gain" },
    { name = "autoLevelSpeed",           label = "Auto-Level" },
}

function TRQEngine.getTunables()
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

function TRQEngine.getTunable(name)
    return HeliConfig.get(name)
end

function TRQEngine.setTunable(name, value)
    HeliConfig.set(name, value)
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Sandbox Options
-------------------------------------------------------------------------------------

function TRQEngine.getSandboxOptions()
    local PARAMS = HeliConfig.getParamDefs()
    local options = {}
    for _, p in pairs(PARAMS) do
        if p.field and p.ns == "TRQ" then
            options[#options + 1] = {
                field = p.field, type = "double",
                default = p.default, min = p.min, max = p.max, desc = p.desc,
            }
        end
    end
    return { namespace = "TRQ", options = options }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Debug
-------------------------------------------------------------------------------------

local DEBUG_COLUMNS = {
    -- Horizontal flight model
    "simPosX", "simPosZ", "simVelX", "simVelZ",
    "errX", "errZ", "errRateX", "errRateZ",
    "desiredVelX", "desiredVelZ", "targetVelY",
    "angleZ", "angleX", "fwdX", "fwdZ",
    -- TRQ angular PD (world-frame torque for readability + body-frame actual)
    "torqueX", "torqueY", "torqueZ",
    "bodyTorqueX", "bodyTorqueY", "bodyTorqueZ",
    "omegaX", "omegaY", "omegaZ",
    "angErrMag",
    -- TRQ desired vs actual (tracking quality)
    "desAngleX", "desAngleY", "desAngleZ",
    "actAngleX", "actAngleY", "actAngleZ",
    -- Up-vectors
    "actUpX", "actUpY", "actUpZ",
    "desUpX", "desUpY", "desUpZ",
    -- Controller internals (actual values used for PD computation)
    "ctrlDesYaw", "ctrlActYaw", "ctrlErrY",
    "ctrlWOmX", "ctrlWOmY", "ctrlWOmZ",
    -- Heading-corrected inertia
    "effIwx", "effIwz",
    -- Torque pipeline diagnostics
    "corrFX", "corrFZ", "vertForce",
    "pitchDelta", "rollDelta", "yawLead",
    -- Rate controller internals
    "rateCmdX", "rateCmdZ", "integralX", "integralZ",
}

function TRQEngine.getDebugColumns()
    return DEBUG_COLUMNS
end

function TRQEngine.getDebugState()
    local simPosX, simPosZ, simVelX, simVelZ = _sim:getState()
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local Ix, Iy, Iz, inertiaValid = TRQTorqueController.getInertia()
    return {
        -- Horizontal flight model
        simPosX = simPosX, simPosZ = simPosZ,
        simVelX = simVelX, simVelZ = simVelZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
        desiredVelX = _dbg.desiredHX, desiredVelZ = _dbg.desiredHZ,
        targetVelY = _dbg.targetVelY,
        angleZ = _dbg.angleZ, angleX = _dbg.angleX,
        fwdX = _dbg.fwdX, fwdZ = _dbg.fwdZ,
        -- TRQ angular PD
        torqueX = _dbg.torqueX, torqueY = _dbg.torqueY, torqueZ = _dbg.torqueZ,
        bodyTorqueX = _dbg.bodyTorqueX, bodyTorqueY = _dbg.bodyTorqueY, bodyTorqueZ = _dbg.bodyTorqueZ,
        omegaX = _dbg.omegaX, omegaY = _dbg.omegaY, omegaZ = _dbg.omegaZ,
        angErrMag = _dbg.angErrMag,
        -- Desired vs actual angles
        desAngleX = _dbg.desAngleX, desAngleY = _dbg.desAngleY, desAngleZ = _dbg.desAngleZ,
        actAngleX = _dbg.actAngleX, actAngleY = _dbg.actAngleY, actAngleZ = _dbg.actAngleZ,
        -- Up-vectors
        actUpX = _dbg.actUpX, actUpY = _dbg.actUpY, actUpZ = _dbg.actUpZ,
        desUpX = _dbg.desUpX, desUpY = _dbg.desUpY, desUpZ = _dbg.desUpZ,
        -- Controller internals
        ctrlDesYaw = _dbg.ctrlDesYaw, ctrlActYaw = _dbg.ctrlActYaw, ctrlErrY = _dbg.ctrlErrY,
        ctrlWOmX = _dbg.ctrlWOmX, ctrlWOmY = _dbg.ctrlWOmY, ctrlWOmZ = _dbg.ctrlWOmZ,
        effIwx = _dbg.effIwx, effIwz = _dbg.effIwz,
        corrFX = _dbg.corrFX, corrFZ = _dbg.corrFZ, vertForce = _dbg.vertForce,
        pitchDelta = _dbg.pitchDelta, rollDelta = _dbg.rollDelta, yawLead = _dbg.yawLead,
        rateCmdX = _dbg.rateCmdX, rateCmdZ = _dbg.rateCmdZ,
        integralX = _dbg.integralX, integralZ = _dbg.integralZ,
        -- Inertia (not in columns — available via /hef inertia command)
        Ix = Ix, Iy = Iy, Iz = Iz, inertiaValid = inertiaValid,
    }
end

function TRQEngine.getIntendedYaw()
    return TRQYawController.getSimYaw() or 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Commands
-------------------------------------------------------------------------------------

function TRQEngine.getCommands()
    return {
        { name = "inertia", args = "", description = "Show computed inertia tensor" },
    }
end

function TRQEngine.executeCommand(name, argsString)
    if name == "inertia" then
        local Ix, Iy, Iz, valid = TRQTorqueController.getInertia()
        if valid then
            return string.format("Inertia: Ix=%.1f Iy=%.1f Iz=%.1f (pitch/yaw/roll)", Ix, Iy, Iz)
        else
            return "Inertia not yet computed (enter vehicle first)"
        end
    end
    return "Unknown TRQ command: " .. tostring(name)
end

-------------------------------------------------------------------------------------
-- Register with framework
-------------------------------------------------------------------------------------
if IFlightEngine then
    IFlightEngine.register("TRQ", TRQEngine)
else
    local function _deferredRegister()
        IFlightEngine.register("TRQ", TRQEngine)
        Events.OnGameStart.Remove(_deferredRegister)
    end
    Events.OnGameStart.Add(_deferredRegister)
end
