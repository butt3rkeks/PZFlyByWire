--[[
    TRQEngine — Torque-based flight engine implementing IFlightEngine

    Research prototype: replaces setAngles teleport with couple-force torque
    so Bullet handles rotation natively (interpolated, no jitter).

    Reuses FBW modules for everything except step 5 (rotation actuator):
      FBW step 5:  ctx.setAngles(FBWOrientation.toEuler())
      TRQ step 5:  estimate omega → PD torque → couple forces

    Registers itself at file scope: IFlightEngine.register("TRQ", TRQEngine)
]]

TRQEngine = {}

-- Same constants as FBW (vertical velocity smoothing, adaptive gain)
TRQEngine.VERTICAL_VELOCITY_SMOOTHING = 0.3
TRQEngine.ADAPTIVE_GAIN_ALPHA   = 0.05
TRQEngine.ADAPTIVE_GAIN_MIN     = 1.0
TRQEngine.ADAPTIVE_GAIN_MAX     = 8.0
TRQEngine.ADAPTIVE_GAIN_DEADZONE = 0.3

-------------------------------------------------------------------------------------
-- Engine state (mirrors FBWEngine)
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
-- _wasGroundMode removed: TRQ uses torque in ground mode (no setAngles discontinuity)
local _smoothedVelY = 0
local _adaptiveGainMultiplier = 1.0

-- Debug state (persisted for getDebugState / recorder)
local _lastTorqueX = 0
local _lastTorqueY = 0
local _lastTorqueZ = 0
local _lastOmegaX = 0
local _lastOmegaY = 0
local _lastOmegaZ = 0
local _lastAngErrMag = 0
-- Flight pipeline state (shared with FBW columns)
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

    FBWOrientation.reset()
    FBWYawController.reset()
    FBWTiltResolver.reset()
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

    -- 1. Init FBWOrientation from vehicle if not initialized
    if not FBWOrientation.isInitialized() then
        FBWOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- 2. FBWInputProcessor: keys → rotation deltas
    local pitchDelta, yawDelta, rollDelta, isRotating = FBWInputProcessor.computeRotationDeltas(
        keys, fpsMultiplier, heliType, blocked, freeMode)

    -- 3. Apply tilt + yaw to FBWOrientation (desired orientation)
    FBWOrientation.applyTilt(pitchDelta, rollDelta)
    FBWOrientation.applyYaw(yawDelta)

    -- 4. Yaw MPC: track intended heading, hard lock when not rotating
    local simYaw = FBWYawController.update(FBWOrientation.getYaw(), isRotating, yawDelta)
    if not isRotating then
        FBWOrientation.setYaw(simYaw)
    end

    -- 5. TRQ: apply torque instead of setAngles
    --    Desired quaternion direct from FBWOrientation (no Euler round-trip).
    --    Desired yaw scalar from FBWOrientation (for scalar yaw PD).
    --    TRQTorqueController: tilt-decomposed pitch/roll + scalar yaw → torque.
    --
    --    Read actual up-vector directly from Bullet (vehicle:getUpVector).
    --    Bypasses Euler angles entirely for tilt — no gimbal flip possible.
    --    Euler Y (heading) is still used for scalar yaw PD (Y is continuous,
    --    no flip at ±90° heading).
    local actUpVec = vehicle:getUpVector(ctx.scratchVector)
    local actUpX = HeliUtil.toLuaNum(actUpVec:x())
    local actUpY = HeliUtil.toLuaNum(actUpVec:y())
    local actUpZ = HeliUtil.toLuaNum(actUpVec:z())

    -- Extract actual yaw from forward vector (same method as FBWOrientation.initFromVehicle).
    -- Euler Y from getAngleY() can be unreliable during Euler flip (XYZ decomposition
    -- Y range shifts). Forward vector projection is always correct.
    local actFwdVec = vehicle:getForwardVector(ctx.scratchVector)
    local actFwdX = HeliUtil.toLuaNum(actFwdVec:x())
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))

    local desQuat = FBWOrientation.getQuaternion()
    -- Extract desired yaw from desired QUATERNION using same atan2 as actual.
    -- FBWOrientation.getYaw() returns _yawDeg which uses OPPOSITE convention
    -- to getForwardVector's atan2: D key decreases _yawDeg but increases physical
    -- heading. In FBW this doesn't matter (setAngles teleports). In TRQ the mismatch
    -- causes the yaw error to grow instead of shrink → sustained oscillation.
    -- Forward vector from desired quaternion: Z-axis (column 2 of rotation matrix)
    -- = same atan2(fwdX, fwdZ) convention as actual heading from getForwardVector
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
          effIwx, effIwz =
        TRQTorqueController.compute(
            desQuat, desYawDeg,
            actUpX, actUpY, actUpZ, actYawDeg,
            omegaY, dt)

    -- NO substep multiplier. The force acts for one 0.01s substep regardless of
    -- frame substep count. FBW KNOWLEDGE.md: "Multiplying by subSteps caused a
    -- subSteps² effect" — the alternating 1/2 substeps create ±100% gain variation
    -- that pumps energy into tilt oscillation (parametric excitation). PD gains are
    -- tuned for the actual per-substep response instead.
    TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)

    -- Persist for debug
    local desAngleX, desAngleY, desAngleZ = FBWOrientation.toEuler()
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
    _lastActUpX = actUpX
    _lastActUpY = actUpY
    _lastActUpZ = actUpZ
    local desUpQ = FBWOrientation.getQuaternion()
    _lastDesUpX = 2 * (desUpQ.x * desUpQ.y + desUpQ.w * desUpQ.z)
    _lastDesUpY = 1 - 2 * (desUpQ.x * desUpQ.x + desUpQ.z * desUpQ.z)
    _lastDesUpZ = 2 * (desUpQ.y * desUpQ.z - desUpQ.w * desUpQ.x)
    _lastActAngleX = ctx.angleX  -- raw Euler (for CSV readability)
    _lastActAngleY = ctx.angleY
    _lastActAngleZ = ctx.angleZ
    _lastDesAngleX = desAngleX
    _lastDesAngleY = desAngleY
    _lastDesAngleZ = desAngleZ

    -- 6. Read forward direction + body angles from desired orientation
    -- (FBW reads from FBWOrientation for the flight model — same here)
    local fwdX, fwdZ = FBWOrientation.getForward()
    local angleZ = FBWOrientation.getBodyPitch()
    local angleX = FBWOrientation.getBodyRoll()
    _lastAngleZ = angleZ
    _lastAngleX = angleX
    _lastFwdX = fwdX
    _lastFwdZ = fwdZ

    -- 7-8. Wall pre-blocking + FBWFilters pipeline → desired horizontal velocity
    local posX = ctx.posX
    local posZ = ctx.posZ
    local totalVelX, totalVelZ, totalSpeed, totalTiltRad, isBlockedHit =
        FBWTiltResolver.resolve(angleZ, angleX, blocked, fwdX, fwdZ, posX, posZ)

    -- 9. Tilt/input flags
    local noiseFloor = HeliConfig.TILT_NOISE_FLOOR
    local noInput = (totalTiltRad < noiseFloor * 2) or (totalSpeed < HeliConfig.NO_INPUT_SPEED_THRESHOLD)
    local hasHInput = not noInput
    _hasTiltInput = hasHInput
    _hasHorizontalInput = hasHInput

    -- 10. FA-off coast logic → resolve desired velocity for sim
    local desiredHX, desiredHZ
    desiredHX, desiredHZ, hasHInput = FBWSimController.resolveDesiredVelocity(
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
    local effectiveInertia = baseBrake * (hasHInput and HeliConfig.GetAccel() or HeliConfig.GetDecel())

    -- 12-14. Sim advance + heading reanchor + soft anchor
    FBWSimController.advanceAndAnchor(_sim, _errorTracker, desiredHX, desiredHZ,
        deltaTime, effectiveInertia, hasHInput, posX, posZ, fps, _flightAssistOff, ctx.positionDeltaSpeed)

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
    local targetVelY, gravComp, vBraking, engineDead = FBWFlightModel.computeVerticalTarget(ctx, freeMode)

    -- Landing zone taper
    if targetVelY < 0 and currentAltitude < groundLevelZ + HeliConfig.LANDING_ZONE_HEIGHT then
        local landingFactor = math.max((currentAltitude - groundLevelZ) / HeliConfig.LANDING_ZONE_HEIGHT, 0)
        landingFactor = math.max(landingFactor, HeliConfig.LANDING_MIN_SPEED_FACTOR)
        targetVelY = targetVelY * landingFactor
    end

    -- Persist flight model outputs for debug
    _lastDesiredHX = desiredHX
    _lastDesiredHZ = desiredHZ
    _lastTargetVelY = targetVelY

    -- 18. Dual-path activation
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local actualHorizontalSpeed = VelocityUtil.horizontalSpeed(velX, velZ)
    local dualPathActive = TRQEngine.isWarmedUp() and
        (hasHInput or errMag > HeliConfig.DUAL_PATH_ERROR_THRESHOLD or actualHorizontalSpeed > HeliConfig.DUAL_PATH_SPEED_THRESHOLD)

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
    local verticalForce = FBWForceComputer.computeThrustForce(
        targetVelY, _smoothedVelY, ctx.mass, verticalGain, gravity,
        ctx.subSteps, ctx.physicsDelta, gravComp)
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

    if not FBWOrientation.isInitialized() then
        FBWOrientation.initFromVehicle(ctx.angleX, ctx.angleY, ctx.angleZ)
    end

    -- Read actual up-vector and heading from Bullet (same as update())
    local actUpVec = vehicle:getUpVector(ctx.scratchVector)
    local actUpX = HeliUtil.toLuaNum(actUpVec:x())
    local actUpY = HeliUtil.toLuaNum(actUpVec:y())
    local actUpZ = HeliUtil.toLuaNum(actUpVec:z())
    local actFwdVec = vehicle:getForwardVector(ctx.scratchVector)
    local actFwdX = HeliUtil.toLuaNum(actFwdVec:x())
    local actFwdZ = HeliUtil.toLuaNum(actFwdVec:z())
    local actYawDeg = math.deg(math.atan2(actFwdX, actFwdZ))

    local desQuat = FBWOrientation.getQuaternion()
    -- Extract desired yaw from quaternion (same atan2 convention as actual)
    local desFwdX = 2 * (desQuat.x * desQuat.z + desQuat.w * desQuat.y)
    local desFwdZ = 1 - 2 * (desQuat.x * desQuat.x + desQuat.y * desQuat.y)
    local desYawDeg = math.deg(math.atan2(desFwdX, desFwdZ))

    local dt = 1.0 / ctx.fps
    local omegaX, omegaY, omegaZ = TRQAngularEstimator.update(ctx.angleX, ctx.angleY, ctx.angleZ, dt)
    local torqueX, torqueY, torqueZ = TRQTorqueController.compute(
        desQuat, desYawDeg,
        actUpX, actUpY, actUpZ, actYawDeg,
        omegaY, dt)

    local subStepScale = math.max(ctx.subSteps, 1)
    TRQCoupleForce.apply(vehicle,
        torqueX * subStepScale, torqueY * subStepScale, torqueZ * subStepScale)

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
            local thrustY = FBWForceComputer.computeThrustForce(
                ascendSpeed, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, true)
            local groundHold = (1.0 - t) * HeliConfig.GROUND_VELOCITY_KILL
            ctx.applyForce(
                -velX * mass * groundHold,
                thrustY,
                -velZ * mass * groundHold)
        end
        liftoff = true

    elseif inTransition then
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GROUND_VELOCITY_THRESHOLD then
            local killFactor = HeliConfig.GROUND_VELOCITY_KILL * (1.0 - t)
            ctx.applyForce(
                -velX * mass * killFactor,
                0,
                -velZ * mass * killFactor)
        end
        if ctx.subSteps > 0 then
            local freeMode = ctx.vehicle:getModData().AutoBalance == true
            local targetVelY, gravComp = FBWFlightModel.computeVerticalTarget(ctx, freeMode)
            if targetVelY < 0 and ctx.currentAltitude < ctx.groundLevelZ + HeliConfig.LANDING_ZONE_HEIGHT then
                local landingFactor = math.max((ctx.currentAltitude - ctx.groundLevelZ) / HeliConfig.LANDING_ZONE_HEIGHT, 0)
                landingFactor = math.max(landingFactor, HeliConfig.LANDING_MIN_SPEED_FACTOR)
                targetVelY = targetVelY * landingFactor
            end
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local forceY = FBWForceComputer.computeThrustForce(
                targetVelY, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, gravComp)
            ctx.applyForce(0, forceY, 0)
        end

    else
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)
        if groundVelMag > HeliConfig.GROUND_VELOCITY_THRESHOLD then
            ctx.applyForce(
                -velX * mass * HeliConfig.GROUND_VELOCITY_KILL,
                -velY * mass * HeliConfig.GROUND_VELOCITY_KILL,
                -velZ * mass * HeliConfig.GROUND_VELOCITY_KILL)
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

    local fx, fz = FBWForceComputer.computeCorrectionForce(
        errX, errZ, errRateX, errRateZ, errMag,
        HeliConfig.GetPositionProportionalGain(), HeliConfig.GetPositionDerivativeGain(),
        cctx.velX, cctx.velZ, cctx.mass, HeliConfig.VEL_FORCE_FACTOR,
        HeliConfig.GetFinalStopDampingGain(), _flightAssistOff,
        HeliConfig.FA_OFF_DEADZONE, HeliConfig.FA_OFF_MIN_DAMPING_SPEED)

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
    -- Shared horizontal PD (reused from FBW params)
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
    -- Horizontal flight model (shared with FBW)
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
        -- Inertia (not in columns — available via /hef inertia command)
        Ix = Ix, Iy = Iy, Iz = Iz, inertiaValid = inertiaValid,
    }
end

function TRQEngine.getIntendedYaw()
    return FBWYawController.getSimYaw() or 0
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
