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
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _tireInflationSet = false
-- _wasGroundMode removed: TRQ uses torque in ground mode (no setAngles discontinuity)
local _smoothedVelY = 0
local _adaptiveGainMultiplier = 1.0
local _rampedTargetVelY = 0  -- smoothed vertical target (prevents 220kN force spikes)

-- Debug state (persisted for getDebugState / recorder)
local _lastTorqueX = 0
local _lastTorqueY = 0
local _lastTorqueZ = 0
local _lastOmegaX = 0
local _lastOmegaY = 0
local _lastOmegaZ = 0
local _lastAngErrMag = 0
-- Flight pipeline state
local _lastDesiredHX = 0
local _lastDesiredHZ = 0
local _lastTargetVelY = 0
local _lastAngleZ = 0
local _lastAngleX = 0
local _lastFwdX = 0
local _lastFwdZ = 0
-- Body-frame torques (what couple forces actually receive)
local _lastBodyTorqueX = 0
local _lastBodyTorqueY = 0
local _lastBodyTorqueZ = 0
-- Controller internals (for diagnosing sign issues)
local _lastCtrlDesYaw = 0
local _lastCtrlActYaw = 0
local _lastCtrlErrY = 0
local _lastCtrlWOmX = 0
local _lastCtrlWOmY = 0
local _lastCtrlWOmZ = 0
local _lastEffIwx = 0
local _lastEffIwz = 0
local _lastCorrFX = 0
local _lastCorrFZ = 0
local _lastVertForce = 0
local _lastPitchDelta = 0
local _lastRollDelta = 0
local _lastYawLead = 0
local _lastRateCmdX = 0
local _lastRateCmdZ = 0
local _lastIntegralX = 0
local _lastIntegralZ = 0
-- TRQ-specific: desired vs actual angles for tracking analysis
local _lastActUpX = 0
local _lastActUpY = 0
local _lastActUpZ = 0
local _lastDesUpX = 0
local _lastDesUpY = 0
local _lastDesUpZ = 0
local _lastDesAngleX = 0
local _lastDesAngleY = 0
local _lastDesAngleZ = 0
local _lastActAngleX = 0
local _lastActAngleY = 0
local _lastActAngleZ = 0

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
    _rampedTargetVelY = 0
    _adaptiveGainMultiplier = 1.0

    _lastTorqueX = 0
    _lastTorqueY = 0
    _lastTorqueZ = 0
    _lastBodyTorqueX = 0
    _lastBodyTorqueY = 0
    _lastBodyTorqueZ = 0
    _lastCtrlDesYaw = 0
    _lastCtrlActYaw = 0
    _lastCtrlErrY = 0
    _lastCtrlWOmX = 0
    _lastCtrlWOmY = 0
    _lastCtrlWOmZ = 0
    _lastEffIwx = 0
    _lastEffIwz = 0
    _lastCorrFX = 0; _lastCorrFZ = 0
    _lastVertForce = 0
    _lastPitchDelta = 0; _lastRollDelta = 0
    _lastYawLead = 0
    _lastRateCmdX = 0; _lastRateCmdZ = 0
    _lastIntegralX = 0; _lastIntegralZ = 0
    _lastOmegaX = 0
    _lastOmegaY = 0
    _lastOmegaZ = 0
    _lastAngErrMag = 0
    _lastDesiredHX = 0
    _lastDesiredHZ = 0
    _lastTargetVelY = 0
    _lastAngleZ = 0
    _lastAngleX = 0
    _lastFwdX = 0
    _lastFwdZ = 0
    _lastActUpX = 0
    _lastActUpY = 0
    _lastActUpZ = 0
    _lastDesUpX = 0
    _lastDesUpY = 0
    _lastDesUpZ = 0
    _lastDesAngleX = 0
    _lastDesAngleY = 0
    _lastDesAngleZ = 0
    _lastActAngleX = 0
    _lastActAngleY = 0
    _lastActAngleZ = 0

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
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))
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

    -- Yaw (A/D)
    if keys.a then yawDelta = HeliConfig.GetYawRotationSpeed() * fpsMultiplier; isRotating = true end
    if keys.d then yawDelta = -HeliConfig.GetYawRotationSpeed() * fpsMultiplier; isRotating = true end

    -- Pitch (UP/DOWN) — key input only, no auto-level
    if keys.up and not keys.left and not keys.right then
        local bodyPitch = TRQOrientation.getBodyPitch()
        if bodyPitch < angle_90 + maxSpeed and not blocked.up then
            pitchDelta = basicAccelRate * fpsMultiplier
        end
    elseif keys.down and not keys.left and not keys.right then
        local bodyPitch = TRQOrientation.getBodyPitch()
        if bodyPitch > angle_90 - maxSpeed and not blocked.down then
            pitchDelta = -basicAccelRate * fpsMultiplier
        end
    end
    -- NO auto-level on pitch release: PD handles return-to-level via desired=level target

    -- Roll (LEFT/RIGHT) — key input only, no auto-level
    if keys.left and not keys.up and not keys.down then
        local bodyRoll = TRQOrientation.getBodyRoll()
        if bodyRoll < angle_90 + maxSpeed and not blocked.left then
            rollDelta = -basicAccelRate * fpsMultiplier
        end
    elseif keys.right and not keys.up and not keys.down then
        local bodyRoll = TRQOrientation.getBodyRoll()
        if bodyRoll > angle_90 - maxSpeed and not blocked.right then
            rollDelta = basicAccelRate * fpsMultiplier
        end
    end
    -- NO auto-level on roll release: PD handles return-to-level

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
    local torqueX, torqueY, torqueZ, angErrMag,
          ctrlDesYaw, ctrlActYaw, ctrlErrY, ctrlWOmX, ctrlWOmY, ctrlWOmZ,
          effIwx, effIwz,
          rateCmdX, rateCmdZ, integralX, integralZ =
        TRQTorqueController.compute(
            desUpX, desUpY, desUpZ, desYawDeg,
            actUpX, actUpY, actUpZ, actYawDeg,
            omegaY, dt, ctx.subSteps)

    -- NO substep multiplier. The force acts for one 0.01s substep regardless of
    -- frame substep count. FBW KNOWLEDGE.md: "Multiplying by subSteps caused a
    -- subSteps² effect" — the alternating 1/2 substeps create ±100% gain variation
    -- that pumps energy into tilt oscillation (parametric excitation). PD gains are
    -- tuned for the actual per-substep response instead.
    TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)

    -- Persist for debug
    local desAngleX, desAngleY, desAngleZ = TRQOrientation.toEuler()
    _lastTorqueX = torqueX  -- world-frame (= actual applied torque)
    _lastTorqueY = torqueY
    _lastTorqueZ = torqueZ
    _lastBodyTorqueX = torqueX  -- same as world (uniform inertia, no body transform)
    _lastBodyTorqueY = torqueY
    _lastBodyTorqueZ = torqueZ
    _lastOmegaX = omegaX
    _lastOmegaY = omegaY
    _lastOmegaZ = omegaZ
    _lastAngErrMag = angErrMag
    _lastCtrlDesYaw = ctrlDesYaw or 0
    _lastCtrlActYaw = ctrlActYaw or 0
    _lastCtrlErrY = ctrlErrY or 0
    _lastCtrlWOmX = ctrlWOmX or 0
    _lastCtrlWOmY = ctrlWOmY or 0
    _lastCtrlWOmZ = ctrlWOmZ or 0
    _lastEffIwx = effIwx or 0
    _lastEffIwz = effIwz or 0
    _lastPitchDelta = pitchDelta
    _lastRollDelta = rollDelta
    _lastRateCmdX = rateCmdX or 0
    _lastRateCmdZ = rateCmdZ or 0
    _lastIntegralX = integralX or 0
    _lastIntegralZ = integralZ or 0
    _lastYawLead = wrapAngle(rawDesYawDeg - actYawDeg)
    _lastActUpX = actUpX
    _lastActUpY = actUpY
    _lastActUpZ = actUpZ
    _lastDesUpX = desUpX  -- tilt-only up-vector (heading-independent)
    _lastDesUpY = desUpY
    _lastDesUpZ = desUpZ
    _lastActAngleX = ctx.angleX  -- raw Euler (for CSV readability)
    _lastActAngleY = ctx.angleY
    _lastActAngleZ = ctx.angleZ
    _lastDesAngleX = desAngleX
    _lastDesAngleY = desAngleY
    _lastDesAngleZ = desAngleZ

    -- 6. Read forward direction + body angles from desired orientation
    -- Read body angles from TRQOrientation for the flight model
    local fwdX, fwdZ = TRQOrientation.getForward()
    local angleZ = TRQOrientation.getBodyPitch()
    local angleX = TRQOrientation.getBodyRoll()
    _lastAngleZ = angleZ
    _lastAngleX = angleX
    _lastFwdX = fwdX
    _lastFwdZ = fwdZ

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
    _lastDesiredHX = desiredHX
    _lastDesiredHZ = desiredHZ
    _lastTargetVelY = targetVelY

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
    _lastVertForce = verticalForce
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
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))
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
        omegaY, dt, ctx.subSteps)

    -- NO substep multiplier — same as airborne mode. The parametric excitation bug
    -- (alternating 1/2 substeps create ±100% gain variation) was fixed in airborne
    -- but this ground mode path was missed. Couple forces act for exactly one 0.01s
    -- substep regardless of frame substep count.
    TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)

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

    _lastCorrFX = fx
    _lastCorrFZ = fz
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
        desiredVelX = _lastDesiredHX, desiredVelZ = _lastDesiredHZ,
        targetVelY = _lastTargetVelY,
        angleZ = _lastAngleZ, angleX = _lastAngleX,
        fwdX = _lastFwdX, fwdZ = _lastFwdZ,
        -- TRQ angular PD
        torqueX = _lastTorqueX, torqueY = _lastTorqueY, torqueZ = _lastTorqueZ,
        bodyTorqueX = _lastBodyTorqueX, bodyTorqueY = _lastBodyTorqueY, bodyTorqueZ = _lastBodyTorqueZ,
        omegaX = _lastOmegaX, omegaY = _lastOmegaY, omegaZ = _lastOmegaZ,
        angErrMag = _lastAngErrMag,
        -- Desired vs actual angles
        desAngleX = _lastDesAngleX, desAngleY = _lastDesAngleY, desAngleZ = _lastDesAngleZ,
        actAngleX = _lastActAngleX, actAngleY = _lastActAngleY, actAngleZ = _lastActAngleZ,
        -- Up-vectors
        actUpX = _lastActUpX, actUpY = _lastActUpY, actUpZ = _lastActUpZ,
        desUpX = _lastDesUpX, desUpY = _lastDesUpY, desUpZ = _lastDesUpZ,
        -- Controller internals
        ctrlDesYaw = _lastCtrlDesYaw, ctrlActYaw = _lastCtrlActYaw, ctrlErrY = _lastCtrlErrY,
        ctrlWOmX = _lastCtrlWOmX, ctrlWOmY = _lastCtrlWOmY, ctrlWOmZ = _lastCtrlWOmZ,
        effIwx = _lastEffIwx, effIwz = _lastEffIwz,
        corrFX = _lastCorrFX, corrFZ = _lastCorrFZ, vertForce = _lastVertForce,
        pitchDelta = _lastPitchDelta, rollDelta = _lastRollDelta, yawLead = _lastYawLead,
        rateCmdX = _lastRateCmdX, rateCmdZ = _lastRateCmdZ,
        integralX = _lastIntegralX, integralZ = _lastIntegralZ,
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
