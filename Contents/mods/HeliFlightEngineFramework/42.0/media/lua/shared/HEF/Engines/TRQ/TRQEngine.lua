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
-------------------------------------------------------------------------------------
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _smoothedVelY = 0
local _adaptiveGainMultiplier = 1.0

-- TRQ-specific debug state (persisted for getDebugState)
local _lastTorqueX = 0
local _lastTorqueY = 0
local _lastTorqueZ = 0
local _lastOmegaX = 0
local _lastOmegaY = 0
local _lastOmegaZ = 0
local _lastAngErrMag = 0

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
    _lastOmegaX = 0
    _lastOmegaY = 0
    _lastOmegaZ = 0
    _lastAngErrMag = 0

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
    --    FBWOrientation holds the desired orientation.
    --    TRQTorqueController computes PD torque from desired vs actual.
    --    TRQCoupleForce applies it via offset impulses.
    local dt = 1.0 / ctx.fps
    local omegaX, omegaY, omegaZ = TRQAngularEstimator.update(ctx.angleX, ctx.angleY, ctx.angleZ, dt)
    local torqueX, torqueY, torqueZ, angErrMag = TRQTorqueController.compute(
        ctx.angleX, ctx.angleY, ctx.angleZ, omegaX, omegaY, omegaZ)
    TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)

    -- Persist for debug
    _lastTorqueX = torqueX
    _lastTorqueY = torqueY
    _lastTorqueZ = torqueZ
    _lastOmegaX = omegaX
    _lastOmegaY = omegaY
    _lastOmegaZ = omegaZ
    _lastAngErrMag = angErrMag

    -- 6. Read forward direction + body angles from desired orientation
    -- (FBW reads from FBWOrientation for the flight model — same here)
    local fwdX, fwdZ = FBWOrientation.getForward()
    local angleZ = FBWOrientation.getBodyPitch()
    local angleX = FBWOrientation.getBodyRoll()

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
-- Uses setAngles on ground (no jitter at ground speed, simpler than fighting
-- Bullet's suspension/wheel physics with torque).
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFGroundResult
function TRQEngine.updateGround(ctx)
    local keys = ctx.keys
    local mass = ctx.mass
    local velX, velY, velZ = ctx.velX, ctx.velY, ctx.velZ
    local heightAboveGround = ctx.currentAltitude - ctx.groundLevelZ

    local BOTTOM = HeliConfig.TRANSITION_ZONE_BOTTOM
    local TOP    = HeliConfig.TRANSITION_ZONE_TOP
    local t = math.max(0, math.min(1, (heightAboveGround - BOTTOM) / (TOP - BOTTOM)))

    local inTransition = (t > 0)
    local liftoff = false

    if inTransition then
        _reinitSim(ctx.posX, ctx.posZ)
        -- Use setAngles on ground (identical to FBW)
        if FBWOrientation.isInitialized() then
            ctx.setAngles(FBWOrientation.toEuler())
        else
            ctx.setAngles(ctx.angleX, ctx.angleY, ctx.angleZ)
        end
    end

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
        keepFlightState = inTransition,
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
    "simPosX", "simPosZ", "simVelX", "simVelZ",
    "errX", "errZ", "errRateX", "errRateZ",
    "desiredVelX", "desiredVelZ", "targetVelY",
    "angleZ", "angleX", "fwdX", "fwdZ",
    -- TRQ-specific
    "torqueX", "torqueY", "torqueZ",
    "omegaX", "omegaY", "omegaZ",
    "angErrMag",
}

function TRQEngine.getDebugColumns()
    return DEBUG_COLUMNS
end

function TRQEngine.getDebugState()
    local simPosX, simPosZ, simVelX, simVelZ = _sim:getState()
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local Ix, Iy, Iz, inertiaValid = TRQTorqueController.getInertia()
    return {
        simPosX = simPosX, simPosZ = simPosZ,
        simVelX = simVelX, simVelZ = simVelZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
        torqueX = _lastTorqueX, torqueY = _lastTorqueY, torqueZ = _lastTorqueZ,
        omegaX = _lastOmegaX, omegaY = _lastOmegaY, omegaZ = _lastOmegaZ,
        angErrMag = _lastAngErrMag,
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
