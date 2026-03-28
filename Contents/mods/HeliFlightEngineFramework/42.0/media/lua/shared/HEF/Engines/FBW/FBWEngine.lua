--[[
    FBWEngine — Fly-by-wire flight engine implementing IFlightEngine

    Wraps all FBW/ modules into the IFlightEngine interface.
    Owns: horizontal flight orchestration, vertical targets, ground behavior.
    Delegates: rotation math, filters, simulation model, error tracking, force computation.

    Registers itself at file scope: IFlightEngine.register("FBW", FBWEngine)
]]

FBWEngine = {}

-- Vertical velocity EMA smoothing alpha (0..1). Lower = more smoothing.
-- Filters Bullet rigid body rocking from the vertical PD controller input
-- so ascend/descend rate stays consistent during horizontal flight.
FBWEngine.VERTICAL_VELOCITY_SMOOTHING = 0.3

-- Adaptive gain: auto-tunes vertical PD gain to compensate for Bullet's
-- hidden velocity damping. Tracks actual vs desired vertical velocity and
-- adjusts a multiplier so observed rate matches the sandbox ascend/descend
-- speed regardless of what Bullet does internally.
FBWEngine.ADAPTIVE_GAIN_ALPHA   = 0.05   -- EMA rate for gain adaptation (slow, stable)
FBWEngine.ADAPTIVE_GAIN_MIN     = 1.0    -- minimum multiplier (no less than base gain)
FBWEngine.ADAPTIVE_GAIN_MAX     = 8.0    -- maximum multiplier (cap runaway)
FBWEngine.ADAPTIVE_GAIN_DEADZONE = 0.3   -- don't adapt when |desiredVelY| < this

-------------------------------------------------------------------------------------
-- Engine state (coordination only — zero domain logic)
-------------------------------------------------------------------------------------
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _smoothedVelY = 0
local _adaptiveGainMultiplier = 1.0

-- Debug state (persisted for getDebugState / recorder)
local _lastDesiredVelX = 0
local _lastDesiredVelZ = 0
local _lastTargetVelY = 0
local _lastAngleZ = 0
local _lastAngleX = 0
local _lastFwdX = 0
local _lastFwdZ = 0

-------------------------------------------------------------------------------------
-- Toolkit instances (Core/ loads before Engines/, so globals are available)
-------------------------------------------------------------------------------------
local _sim = SimModel2D.new(HeliConfig.TARGET_FPS)
local _errorTracker = ErrorTracker2D.new(HeliConfig.HISTORY_SIZE, 5)

-------------------------------------------------------------------------------------
-- IFlightEngine: Metadata
-------------------------------------------------------------------------------------

function FBWEngine.getInfo()
    return { name = "FBW", version = "1.0", description = "Fly-by-wire MPC" }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Lifecycle
-------------------------------------------------------------------------------------

function FBWEngine.resetFlightState()
    _hasTiltInput = false
    _hasHorizontalInput = false
    _flightAssistOff = false
    _warmupCounter = HeliConfig.GetWarmupFrames()
    _simInitialized = false
    _smoothedVelY = 0
    _adaptiveGainMultiplier = 1.0
    _lastDesiredVelX = 0
    _lastDesiredVelZ = 0
    _lastTargetVelY = 0
    _lastAngleZ = 0
    _lastAngleX = 0
    _lastFwdX = 0
    _lastFwdZ = 0

    FBWOrientation.reset()
    FBWYawController.reset()
    FBWTiltResolver.reset()
    _sim:reset(0, 0)
    _errorTracker:reset()
    HeliForceAdapter.resetPhysicsTime()
    HeliVelocityAdapter.resetSmoothing()
end

--- Internal: re-anchor sim model to a known position.
--- Used by initFlight (framework API) and internally during update().
local function _reinitSim(posX, posZ)
    _sim:reset(posX, posZ)
    _simInitialized = true
end

function FBWEngine.initFlight(vehicle)
    local posX = vehicle:getX()
    local posZ = vehicle:getY()
    if posX == nil or posZ == nil then return end
    _reinitSim(HeliUtil.toLuaNum(posX), HeliUtil.toLuaNum(posZ))
end

function FBWEngine.tickWarmup()
    if _warmupCounter > 0 then
        _warmupCounter = _warmupCounter - 1
    end
end

function FBWEngine.isWarmedUp()
    return _warmupCounter <= 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: update(ctx) — one frame of airborne flight
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFUpdateResult
function FBWEngine.update(ctx)
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

    -- 3. Apply tilt + yaw to FBWOrientation
    FBWOrientation.applyTilt(pitchDelta, rollDelta)
    FBWOrientation.applyYaw(yawDelta)

    -- 4. Yaw MPC: track intended heading, hard lock when not rotating
    local simYaw = FBWYawController.update(FBWOrientation.getYaw(), isRotating, yawDelta)
    if not isRotating then
        FBWOrientation.setYaw(simYaw)
    end

    -- 5. setAngles output
    ctx.setAngles(FBWOrientation.toEuler())

    -- 6. Read forward direction + body angles
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

    -- 16. Velocity from framework ctx (read once per frame by HeliMove)
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ

    -- 16b. Smooth velY for the vertical PD controller.
    -- Bullet's rigid body rocking during horizontal flight causes velY to
    -- oscillate wildly frame-to-frame. The raw signal makes the PD chase noise
    -- instead of driving toward target, severely reducing effective vertical rate.
    -- EMA filter (alpha=0.3) gives ~3-frame effective window — fast enough to
    -- track real ascent/descent, smooth enough to reject per-frame rocking.
    local alpha = FBWEngine.VERTICAL_VELOCITY_SMOOTHING
    _smoothedVelY = alpha * velY + (1.0 - alpha) * _smoothedVelY

    -- 17. Vertical target (absorbed from HeliMove)
    local targetVelY, gravComp, vBraking, engineDead = FBWFlightModel.computeVerticalTarget(ctx, freeMode)

    -- Landing zone taper
    if targetVelY < 0 and currentAltitude < groundLevelZ + HeliConfig.LANDING_ZONE_HEIGHT then
        local landingFactor = math.max((currentAltitude - groundLevelZ) / HeliConfig.LANDING_ZONE_HEIGHT, 0)
        landingFactor = math.max(landingFactor, HeliConfig.LANDING_MIN_SPEED_FACTOR)
        targetVelY = targetVelY * landingFactor
    end

    -- Persist for debug
    _lastDesiredVelX = desiredHX
    _lastDesiredVelZ = desiredHZ
    _lastTargetVelY = targetVelY

    -- 18. Dual-path activation
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local actualHorizontalSpeed = VelocityUtil.horizontalSpeed(velX, velZ)
    local dualPathActive = FBWEngine.isWarmedUp() and
        (hasHInput or errMag > HeliConfig.DUAL_PATH_ERROR_THRESHOLD or actualHorizontalSpeed > HeliConfig.DUAL_PATH_SPEED_THRESHOLD)

    -- 19. Adaptive gain: track actual vs desired vertical velocity and adjust
    -- a multiplier so the PD compensates for Bullet's hidden damping.
    -- Only adapts during active ascend/descend (not hover).
    local absTarget = math.abs(targetVelY)
    if absTarget > FBWEngine.ADAPTIVE_GAIN_DEADZONE then
        local absActual = math.abs(_smoothedVelY)
        -- Only adapt once velocity has meaningfully responded (> 10% of target).
        -- This prevents the multiplier from spiking to max on the first frames
        -- of every W/S press when smoothedVelY is still near zero.
        if absActual > absTarget * 0.1 then
            -- Ratio of how much velocity we're achieving vs target.
            -- < 1 means Bullet is eating force → need higher gain.
            -- > 1 means overshooting → reduce gain.
            local ratio = absActual / absTarget
            local desired = math.min(1.0 / ratio, FBWEngine.ADAPTIVE_GAIN_MAX)
            desired = math.max(desired, FBWEngine.ADAPTIVE_GAIN_MIN)
            local a = FBWEngine.ADAPTIVE_GAIN_ALPHA
            _adaptiveGainMultiplier = a * desired + (1.0 - a) * _adaptiveGainMultiplier
        end
    end

    -- 19b. Vertical thrust (uses smoothed velY + adaptive gain)
    local verticalGain = HeliConfig.GetVerticalGain() * _adaptiveGainMultiplier
    local gravity = HeliConfig.GetGravity()
    local verticalForce = FBWForceComputer.computeThrustForce(
        targetVelY, _smoothedVelY, ctx.mass, verticalGain, gravity,
        ctx.subSteps, ctx.physicsDelta, gravComp)
    if verticalForce ~= 0 then
        ctx.applyForce(0, verticalForce, 0)
    end

    -- 20. Display speed from sim velocity
    local displaySpeed = CoordUtil.msToKmh(VelocityUtil.horizontalSpeed(simVelX, simVelZ))

    -- 21. Return results
    return {
        -- Mandatory OUT
        engineDead = engineDead,
        dualPathActive = dualPathActive,
        displaySpeed = displaySpeed,
        -- Framework-consumed (wall damage, telemetry)
        isBlockedHit = isBlockedHit,
        telemetrySpeed = displaySpeed,
        -- Debug / flight data recorder
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
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: updateGround(ctx) — transition-aware ground behavior
--
-- Replaces generic GroundModel with FBW-specific logic that blends ground-hold
-- forces and airborne PD forces across the transition zone. This eliminates
-- the abrupt jolt when crossing the airborne threshold.
--
-- Blend factor t:
--   0   (below TRANSITION_ZONE_BOTTOM) = pure ground hold
--   0→1 (transition zone)              = ground hold fades, FBW thrust active
--   1   (at TRANSITION_ZONE_TOP)       = full airborne (HeliMove switches to update())
--
-- keepFlightState = true in the transition zone preserves sim model, orientation,
-- yaw controller, and error tracker across the ground↔airborne boundary.
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFGroundResult
function FBWEngine.updateGround(ctx)
    local keys = ctx.keys
    local mass = ctx.mass
    local velX, velY, velZ = ctx.velX, ctx.velY, ctx.velZ
    local heightAboveGround = ctx.currentAltitude - ctx.groundLevelZ

    -- Blend factor: 0 = pure ground hold, 1 = full flight authority
    local BOTTOM = HeliConfig.TRANSITION_ZONE_BOTTOM
    local TOP    = HeliConfig.TRANSITION_ZONE_TOP
    local t = math.max(0, math.min(1, (heightAboveGround - BOTTOM) / (TOP - BOTTOM)))

    local inTransition = (t > 0)
    local liftoff = false

    -- Keep sim model anchored to actual position during transition zone
    -- so FBW state is warm and ready when HeliMove switches to update().
    if inTransition then
        _reinitSim(ctx.posX, ctx.posZ)

        -- Hold orientation level during transition — without this, Bullet's
        -- native vehicle physics (wheel/suspension model) takes over rotation
        -- and can flip the helicopter.
        if FBWOrientation.isInitialized() then
            ctx.setAngles(FBWOrientation.toEuler())
        else
            -- Not yet initialized: lock to current angles (prevent Bullet drift)
            ctx.setAngles(ctx.angleX, ctx.angleY, ctx.angleZ)
        end
    end

    if keys.w and ctx.fuelPercent > 0 then
        -- === LIFTOFF ===
        ctx.setPhysicsActive(true)
        if ctx.subSteps > 0 then
            -- Vertical: same PD + gravComp as airborne path (consistent force model)
            local verticalGain = HeliConfig.GetVerticalGain()
            local gravity = HeliConfig.GetGravity()
            local ascendSpeed = HeliConfig.GetAscend()
            local thrustY = FBWForceComputer.computeThrustForce(
                ascendSpeed, velY, mass, verticalGain, gravity,
                ctx.subSteps, ctx.physicsDelta, true)

            -- Horizontal: velocity kill fades out as t rises
            local groundHold = (1.0 - t) * HeliConfig.GROUND_VELOCITY_KILL
            ctx.applyForce(
                -velX * mass * groundHold,
                thrustY,
                -velZ * mass * groundHold)
        end
        liftoff = true

    elseif inTransition then
        -- === TRANSITION ZONE (no W key) ===
        -- Use the same vertical model as airborne: handles S-key descent,
        -- no-fuel fall, engine dead, and hover (no input → desiredVelY=0).
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)

        -- Horizontal: velocity kill fades with altitude
        if groundVelMag > HeliConfig.GROUND_VELOCITY_THRESHOLD then
            local killFactor = HeliConfig.GROUND_VELOCITY_KILL * (1.0 - t)
            ctx.applyForce(
                -velX * mass * killFactor,
                0,
                -velZ * mass * killFactor)
        end

        -- Vertical: full FBW vertical model (S=descend, nothing=hover, etc.)
        if ctx.subSteps > 0 then
            local freeMode = ctx.vehicle:getModData().AutoBalance == true
            local targetVelY, gravComp = FBWFlightModel.computeVerticalTarget(ctx, freeMode)

            -- Apply landing zone taper (same as airborne path)
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
        -- === PURE GROUND (below transition zone, no W key) ===
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
-- IFlightEngine: applyCorrectionForces(vehicle) — 0-frame delay path
-------------------------------------------------------------------------------------
--- @param cctx HEFCorrectionCtx
function FBWEngine.applyCorrectionForces(cctx)
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
-- IFlightEngine: Tunables (runtime, session-only via HeliConfig)
-------------------------------------------------------------------------------------

-- Tunables: only store the name→label mapping. All other metadata (default, min, max)
-- comes from HeliConfig PARAMS — single source of truth.
local TUNABLE_NAMES = {
    { name = "positionProportionalGain", label = "P Gain" },
    { name = "positionDerivativeGain",   label = "D Gain" },
    { name = "maxPositionError",         label = "Max Error" },
    { name = "finalStopDampingGain",     label = "Stop Gain" },
    { name = "yawCorrectionGain",        label = "Yaw Gain" },
    { name = "autoLevelSpeed",           label = "Auto-Level" },
}

function FBWEngine.getTunables()
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

function FBWEngine.getTunable(name)
    return HeliConfig.get(name)
end

function FBWEngine.setTunable(name, value)
    HeliConfig.set(name, value)
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Sandbox Options (persistent, per-save)
-------------------------------------------------------------------------------------

function FBWEngine.getSandboxOptions()
    local PARAMS = HeliConfig.getParamDefs()
    local options = {}
    for _, p in pairs(PARAMS) do
        if p.field and p.ns == "FBW" then
            options[#options + 1] = {
                field = p.field, type = "double",
                default = p.default, min = p.min, max = p.max, desc = p.desc,
            }
        end
    end
    return { namespace = "FBW", options = options }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Debug
-------------------------------------------------------------------------------------

local DEBUG_COLUMNS = {
    "simPosX", "simPosZ", "simVelX", "simVelZ",
    "errX", "errZ", "errRateX", "errRateZ",
    "desiredVelX", "desiredVelZ", "targetVelY",
    "angleZ", "angleX", "fwdX", "fwdZ",
}

function FBWEngine.getDebugColumns()
    return DEBUG_COLUMNS
end

function FBWEngine.getDebugState()
    local simPosX, simPosZ, simVelX, simVelZ = _sim:getState()
    local errX, errZ, errRateX, errRateZ = _errorTracker:getError(HeliConfig.GetMaxPositionError())
    return {
        simPosX = simPosX, simPosZ = simPosZ,
        simVelX = simVelX, simVelZ = simVelZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
        desiredVelX = _lastDesiredVelX, desiredVelZ = _lastDesiredVelZ,
        targetVelY = _lastTargetVelY,
        angleZ = _lastAngleZ, angleX = _lastAngleX,
        fwdX = _lastFwdX, fwdZ = _lastFwdZ,
    }
end

function FBWEngine.getIntendedYaw()
    return FBWYawController.getSimYaw() or 0
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Commands
-------------------------------------------------------------------------------------

function FBWEngine.getCommands()
    return {}
end

function FBWEngine.executeCommand(name, argsString)
    return "Unknown FBW command: " .. tostring(name)
end

-------------------------------------------------------------------------------------
-- Register with framework
-- Guard: if IFlightEngine loaded after us (directory traversal order varies),
-- defer to OnGameStart (all shared/ files are loaded before any events fire).
-------------------------------------------------------------------------------------
if IFlightEngine then
    IFlightEngine.register("FBW", FBWEngine)
else
    local function _deferredRegister()
        IFlightEngine.register("FBW", FBWEngine)
        Events.OnGameStart.Remove(_deferredRegister)
    end
    Events.OnGameStart.Add(_deferredRegister)
end
