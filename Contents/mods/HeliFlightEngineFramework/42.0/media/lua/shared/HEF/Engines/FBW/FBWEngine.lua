--[[
    FBWEngine — Fly-by-wire flight engine implementing IFlightEngine

    Wraps all FBW/ modules into the IFlightEngine interface.
    Owns: horizontal flight orchestration, vertical targets, ground behavior.
    Delegates: rotation math, filters, simulation model, error tracking, force computation.

    Registers itself at file scope: IFlightEngine.register("FBW", FBWEngine)
]]

FBWEngine = {}

-------------------------------------------------------------------------------------
-- Engine state (coordination only — zero domain logic)
-------------------------------------------------------------------------------------
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _prevPosX = nil
local _prevPosZ = nil

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
    _warmupCounter = HeliConfig.WARMUP_FRAMES
    _simInitialized = false
    _prevPosX = nil
    _prevPosZ = nil

    FBWOrientation.reset()
    FBWYawController.reset()
    FBWRawSim.reset(0, 0)
    FBWErrorTracker.reset()
    HeliForceAdapter.resetPhysicsTime()
    HeliVelocityAdapter.resetSmoothing()
end

function FBWEngine.initFlight(vehicle)
    local posX = vehicle:getX()
    local posZ = vehicle:getY()
    if posX == nil or posZ == nil then return end
    local px = HeliUtil.toLuaNum(posX)
    local pz = HeliUtil.toLuaNum(posZ)
    FBWRawSim.reset(px, pz)
    _simInitialized = true
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
    local curr_z = ctx.curr_z
    local nowMaxZ = ctx.nowMaxZ
    local blocked = ctx.blocked

    local toLuaNum = HeliUtil.toLuaNum
    local freeMode = vehicle:getModData().AutoBalance == true
    _flightAssistOff = freeMode

    -- 1. Init FBWOrientation from vehicle if not initialized
    if not FBWOrientation.isInitialized() then
        FBWOrientation.initFromVehicle(
            toLuaNum(vehicle:getAngleX()),
            toLuaNum(vehicle:getAngleY()),
            toLuaNum(vehicle:getAngleZ()))
    end

    -- 2. FBWInputProcessor: keys → rotation deltas
    local ax, ay, az, isRotating = FBWInputProcessor.computeRotationDeltas(
        keys, fpsMultiplier, heliType, blocked, freeMode)

    -- 3. Apply tilt + yaw to FBWOrientation
    FBWOrientation.applyTilt(ax, az)
    FBWOrientation.applyYaw(ay)

    -- 4. Yaw MPC: track intended heading, hard lock when not rotating
    local simYaw = FBWYawController.update(FBWOrientation.getYaw(), isRotating, ay)
    if not isRotating then
        FBWOrientation.setYaw(simYaw)
    end

    -- 5. setAngles output
    vehicle:setAngles(FBWOrientation.toEuler())

    -- 6. Read forward direction + body angles
    local fwdX, fwdZ = FBWOrientation.getForward()
    local angleZ = FBWOrientation.getBodyPitch()
    local angleX = FBWOrientation.getBodyRoll()

    -- 7. Wall pre-blocking: use framework-provided blocked directions
    local angle_90 = math.rad(90)
    local pitchDev = angleZ - angle_90
    local rollDev = angleX - angle_90
    local isBlockedHit = false

    local cosZ = math.cos(angleZ)
    local cosX = math.cos(angleX)
    local pitchBlocked = false
    local rollBlocked = false

    if math.abs(cosZ) > HeliConfig.DIRECTION_COS_THRESHOLD then
        pitchBlocked = (cosZ < 0) and blocked.up or blocked.down
        if pitchBlocked then isBlockedHit = true end
    end
    if math.abs(cosX) > HeliConfig.DIRECTION_COS_THRESHOLD then
        rollBlocked = (cosX < 0) and blocked.right or blocked.left
        if rollBlocked then isBlockedHit = true end
    end

    local effectivePitchDev = pitchBlocked and 0 or pitchDev
    local effectiveRollDev = rollBlocked and 0 or rollDev
    local totalTiltRad = math.sqrt(effectivePitchDev * effectivePitchDev + effectiveRollDev * effectiveRollDev)

    -- 8. FBWFilters pipeline: noiseFloor → thrustDecomposition → speedClamp → directionClamp
    local noiseFloor = HeliConfig.TILT_NOISE_FLOOR
    local maxHSpeed = HeliConfig.get("hspeed")

    local effectiveTilt = FBWFilters.applyNoiseFloor(totalTiltRad, noiseFloor)

    local totalVelX, totalVelZ, totalSpeed = FBWFilters.decomposeThrustDirection(
        effectivePitchDev, effectiveRollDev, totalTiltRad, fwdX, fwdZ, maxHSpeed, effectiveTilt)

    totalVelX, totalVelZ, totalSpeed = FBWFilters.clampSpeed(totalVelX, totalVelZ, maxHSpeed)

    -- Direction clamping: need movement angle from position delta
    local posX = ctx.posX
    local posZ = ctx.posZ
    if _prevPosX and totalSpeed > 0.1 then
        local dx = posX - _prevPosX
        local dz = posZ - _prevPosZ
        local moveMag = math.sqrt(dx * dx + dz * dz)
        if moveMag > 0.08 then
            local moveAngle = math.atan2(dz, dx)
            totalVelX, totalVelZ = FBWFilters.clampThrustDirection(
                totalVelX, totalVelZ, totalSpeed, moveAngle, moveMag, HeliConfig.MAX_THRUST_LEAD)
        end
    end
    _prevPosX = posX
    _prevPosZ = posZ

    -- 9. Tilt/input flags
    local noInput = (totalTiltRad < noiseFloor * 2) or (totalSpeed < HeliConfig.NO_INPUT_SPEED_THRESHOLD)
    local hasHInput = not noInput
    _hasTiltInput = hasHInput
    _hasHorizontalInput = hasHInput

    -- 10. FA-off coast logic
    local desiredHX, desiredHZ = 0, 0
    if hasHInput then
        desiredHX, desiredHZ = totalVelX, totalVelZ
    end

    if freeMode and noInput then
        local _, _, svx, svz = FBWRawSim.getState()
        local actualSpeed = math.sqrt(ctx.velX * ctx.velX + ctx.velZ * ctx.velZ)
        if actualSpeed < HeliConfig.FA_OFF_DEADZONE then
            desiredHX, desiredHZ = 0, 0
            FBWEngine.initFlight(vehicle)
        else
            desiredHX, desiredHZ = svx, svz
        end
        hasHInput = true
        _hasHorizontalInput = true
    end

    -- 11. Select effective inertia
    if not _simInitialized then
        FBWEngine.initFlight(vehicle)
    end

    local fps = ctx.fps
    local dt = 1.0 / fps
    local baseBrake = HeliConfig.get("brake")
    local effectiveInertia = baseBrake * (hasHInput and HeliConfig.get("accel") or HeliConfig.get("decel"))

    -- 12. FBWRawSim.advance
    FBWRawSim.advance(desiredHX, desiredHZ, dt, effectiveInertia)

    -- 13. Heading re-anchor check (skip in FA-off — coast must survive turns)
    if not _flightAssistOff then
        if FBWYawController.checkHeadingReanchor(fps) then
            FBWRawSim.snapPosition(posX, posZ)
            FBWErrorTracker.clearHistory()
        end
    end

    -- 14. Soft anchor: low speed + no input → blend sim toward actual
    local posDeltaSpeed = HeliVelocityAdapter.getPositionDeltaSpeed()
    local desiredMag = math.sqrt(desiredHX * desiredHX + desiredHZ * desiredHZ)
    if desiredMag < HeliConfig.SIM_SNAP_THRESHOLD and not hasHInput and posDeltaSpeed < HeliConfig.SIM_SNAP_THRESHOLD then
        FBWRawSim.blendToward(posX, posZ, HeliConfig.SIM_BLEND_RATE)
    end

    -- 15. Record in error tracker
    local simPosX, simPosZ, simVelX, simVelZ = FBWRawSim.getState()
    FBWErrorTracker.record(posX, posZ, simPosX, simPosZ)

    -- 16. Velocity from framework ctx (read once per frame by HeliMove)
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ

    -- 17. Vertical target (absorbed from HeliMove)
    local targetVelY, gravComp, vBraking, engineDead = FBWFlightModel.computeVerticalTarget(
        vehicle, curr_z, freeMode, keys, velY)

    -- Landing zone taper
    if targetVelY < 0 and curr_z < nowMaxZ + HeliConfig.LANDING_ZONE_HEIGHT then
        local landingFactor = math.max((curr_z - nowMaxZ) / HeliConfig.LANDING_ZONE_HEIGHT, 0)
        landingFactor = math.max(landingFactor, HeliConfig.LANDING_MIN_SPEED_FACTOR)
        targetVelY = targetVelY * landingFactor
    end

    -- 18. Dual-path activation
    local errX, errZ, errRateX, errRateZ = FBWErrorTracker.getPositionError()
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local hSpeedActual = math.sqrt(velX * velX + velZ * velZ)
    local dualPathActive = FBWEngine.isWarmedUp() and
        (hasHInput or errMag > HeliConfig.DUAL_PATH_ERROR_THRESHOLD or hSpeedActual > HeliConfig.DUAL_PATH_SPEED_THRESHOLD)

    -- 19. Vertical thrust
    local Kp = HeliConfig.get("kp")
    local gravity = HeliConfig.get("gravity")
    local fy = FBWForceComputer.computeThrustForce(
        targetVelY, velY, ctx.mass, Kp, gravity,
        ctx.subSteps, ctx.physicsDelta, gravComp)
    if fy ~= 0 then
        HeliForceAdapter.applyForceImmediate(vehicle, 0, fy, 0)
    end

    -- 20. Display speed from sim velocity
    local displaySpeed = math.sqrt(simVelX * simVelX + simVelZ * simVelZ) * 3.6  -- m/s → km/h

    -- 21. Return results
    return {
        -- Mandatory OUT
        engineDead = engineDead,
        dualPathActive = dualPathActive,
        displaySpeed = displaySpeed,
        -- Optional OUT (debug/telemetry)
        ax = ax, ay = ay, az = az,
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
        isBlockedHit = isBlockedHit,
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: updateGround(ctx) — ground behavior
-------------------------------------------------------------------------------------
--- @param ctx HEFCtx
--- @return HEFGroundResult
function FBWEngine.updateGround(ctx)
    local vehicle = ctx.vehicle
    local keys = ctx.keys
    local mass = ctx.mass
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ
    local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)

    local liftoff = false

    if keys.w and vehicle:getRemainingFuelPercentage() > 0 then
        vehicle:setPhysicsActive(true)
        local ascendSpeed = HeliConfig.get("ascend")
        local gravity = HeliConfig.get("gravity")
        if ctx.subSteps > 0 then
            local Kp = HeliConfig.get("kp")
            local liftFy = Kp * (ascendSpeed - velY) * mass * ctx.subSteps + mass * gravity * ctx.subSteps
            HeliForceAdapter.applyForceImmediate(vehicle,
                -velX * mass * HeliConfig.GROUND_VELOCITY_KILL,
                liftFy,
                -velZ * mass * HeliConfig.GROUND_VELOCITY_KILL)
        end
        liftoff = true
    elseif groundVelMag > HeliConfig.GROUND_VELOCITY_THRESHOLD then
        HeliForceAdapter.applyForceImmediate(vehicle,
            -velX * mass * HeliConfig.GROUND_VELOCITY_KILL,
            -velY * mass * HeliConfig.GROUND_VELOCITY_KILL,
            -velZ * mass * HeliConfig.GROUND_VELOCITY_KILL)
    end

    return {
        liftoff = liftoff,
        displaySpeed = 0,
    }
end

-------------------------------------------------------------------------------------
-- IFlightEngine: applyCorrectionForces(vehicle) — 0-frame delay path
-------------------------------------------------------------------------------------
--- @param cctx HEFCorrectionCtx
function FBWEngine.applyCorrectionForces(cctx)
    local errX, errZ, errRateX, errRateZ = FBWErrorTracker.getPositionError()
    local errMag = math.sqrt(errX * errX + errZ * errZ)

    local fx, fz = FBWForceComputer.computeCorrectionForce(
        errX, errZ, errRateX, errRateZ, errMag,
        HeliConfig.get("pgain"), HeliConfig.get("dgain"),
        cctx.velX, cctx.velZ, cctx.mass, HeliConfig.VEL_FORCE_FACTOR,
        HeliConfig.get("fstopgain"), _flightAssistOff,
        HeliConfig.FA_OFF_DEADZONE, HeliConfig.FA_OFF_MIN_DAMPING_SPEED)

    HeliForceAdapter.applyForceImmediate(cctx.vehicle, fx, 0, fz)
end

-------------------------------------------------------------------------------------
-- IFlightEngine: Tunables (runtime, session-only via HeliConfig)
-------------------------------------------------------------------------------------

local TUNABLE_DEFS = {
    { name = "pgain",    label = "P Gain",        default = 7.0,  min = 0.1, max = 50.0 },
    { name = "dgain",    label = "D Gain",        default = 0.3,  min = 0.0, max = 5.0 },
    { name = "maxerr",   label = "Max Error",     default = 10.0, min = 1.0, max = 100.0 },
    { name = "fstopgain", label = "Stop Gain",    default = 0.3,  min = 0.0, max = 2.0 },
    { name = "yawgain",  label = "Yaw Gain",      default = 0.9,  min = 0.0, max = 1.0 },
    { name = "autolevel", label = "Auto-Level",   default = 1.0,  min = 0.1, max = 5.0 },
}

function FBWEngine.getTunables()
    local result = {}
    for _, def in ipairs(TUNABLE_DEFS) do
        result[#result + 1] = {
            name = def.name,
            label = def.label,
            value = HeliConfig.get(def.name),
            min = def.min,
            max = def.max,
            default = def.default,
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
    return {
        namespace = "FBW",
        options = {
            { field = "GravityEstimate",    type = "double", default = 9.8,   min = 5.0,  max = 20.0,   desc = "Gravity estimate (Bullet units/s^2)" },
            { field = "AscendSpeed",        type = "double", default = 8.0,   min = 1.0,  max = 30.0,   desc = "Ascend speed (Bullet Y/s)" },
            { field = "DescendSpeed",       type = "double", default = 14.0,  min = 1.0,  max = 25.0,   desc = "Descend speed (Bullet Y/s)" },
            { field = "GravityFallSpeed",   type = "double", default = 24.5,  min = 2.0,  max = 50.0,   desc = "Engine-off fall speed (Bullet Y/s)" },
            { field = "EngineDeadFallSpeed", type = "double", default = 35.0,  min = 3.0,  max = 60.0,   desc = "Engine-dead fall speed (Bullet Y/s)" },
            { field = "ResponsivenessGain", type = "double", default = 8.0,   min = 1.0,  max = 20.0,   desc = "Vertical PD gain (responsiveness)" },
            { field = "MaxHorizontalSpeed", type = "double", default = 450.0, min = 10.0, max = 1000.0, desc = "Max horizontal speed (m/s)" },
            { field = "BrakingMultiplier",  type = "double", default = 0.05,  min = 0.01, max = 1.0,    desc = "Base inertia rate" },
        },
    }
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
    local simPosX, simPosZ, simVelX, simVelZ = FBWRawSim.getState()
    local errX, errZ, errRateX, errRateZ = FBWErrorTracker.getPositionError()
    return {
        simPosX = simPosX, simPosZ = simPosZ,
        simVelX = simVelX, simVelZ = simVelZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
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
