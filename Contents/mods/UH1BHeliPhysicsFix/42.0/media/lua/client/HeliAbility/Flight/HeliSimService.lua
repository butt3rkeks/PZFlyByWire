--[[
    HeliSimService — Facade: pure orchestration, zero computation

    Wires Core/ modules + Adapters/ + HeliInputProcessor into a single update()
    call for HeliMove. All math is delegated. Facade only coordinates.

    Public API (called by HeliMove):
      update()                — one frame of horizontal flight
      applyCorrectionForces() — 0-frame delay correction path
      applyThrustForces()     — vertical thrust
      resetFlightState()      — new flight session
      initFlight()            — anchor sim to vehicle position
      tickWarmup() / isWarmedUp()

    Accessors (for HeliDebugCommands):
      getPGain/setPGain, getDGain/setDGain, getMaxError/setMaxError,
      getFinalStopGain/setFinalStopGain, getYawGain/setYawGain,
      getAutoLevelMultiplier/setAutoLevelMultiplier, getIntendedYaw,
      hasTiltInput, isFlightAssistOff, hasHorizontalInput, getSimState,
      getPositionError
]]

HeliSimService = {}

-------------------------------------------------------------------------------------
-- Facade state (coordination only — zero domain logic)
-------------------------------------------------------------------------------------
local _hasTiltInput = false
local _hasHorizontalInput = false
local _flightAssistOff = false
local _warmupCounter = 0
local _simInitialized = false
local _prevPosX = nil
local _prevPosZ = nil

-------------------------------------------------------------------------------------
-- Accessors: delegate to HeliConfig for all tunables
-------------------------------------------------------------------------------------
function HeliSimService.getPGain() return HeliConfig.get("pgain") end
function HeliSimService.setPGain(v) HeliConfig.set("pgain", v) end
function HeliSimService.getDGain() return HeliConfig.get("dgain") end
function HeliSimService.setDGain(v) HeliConfig.set("dgain", v) end
function HeliSimService.getMaxError() return HeliConfig.get("maxerr") end
function HeliSimService.setMaxError(v) HeliConfig.set("maxerr", v) end
function HeliSimService.getFinalStopGain() return HeliConfig.get("fstopgain") end
function HeliSimService.setFinalStopGain(v) HeliConfig.set("fstopgain", v) end
function HeliSimService.getYawGain() return HeliConfig.get("yawgain") end
function HeliSimService.setYawGain(v) HeliConfig.set("yawgain", v) end
function HeliSimService.getAutoLevelMultiplier() return HeliConfig.get("autolevel") end
function HeliSimService.setAutoLevelMultiplier(v) HeliConfig.set("autolevel", v) end

function HeliSimService.hasTiltInput() return _hasTiltInput end
function HeliSimService.isFlightAssistOff() return _flightAssistOff end
function HeliSimService.hasHorizontalInput() return _hasHorizontalInput end
function HeliSimService.getIntendedYaw() return HeliYawController.getSimYaw() end

function HeliSimService.getPositionError()
    return HeliErrorTracker.getPositionError()
end

function HeliSimService.getSimState()
    return HeliRawSim.getState()
end

-------------------------------------------------------------------------------------
-- Lifecycle
-------------------------------------------------------------------------------------

function HeliSimService.resetFlightState()
    _hasTiltInput = false
    _hasHorizontalInput = false
    _flightAssistOff = false
    _warmupCounter = HeliConfig.WARMUP_FRAMES
    _simInitialized = false
    _prevPosX = nil
    _prevPosZ = nil

    HeliOrientation.reset()
    HeliYawController.reset()
    HeliRawSim.reset(0, 0)
    HeliErrorTracker.reset()
    HeliForceAdapter.resetPhysicsTime()
    HeliVelocityAdapter.resetSmoothing()
end

function HeliSimService.initFlight(vehicle)
    local posX = vehicle:getX()
    local posZ = vehicle:getY()
    if posX == nil or posZ == nil then return end
    local px = HeliUtil.toLuaNum(posX)
    local pz = HeliUtil.toLuaNum(posZ)
    HeliRawSim.reset(px, pz)
    _simInitialized = true
end

function HeliSimService.tickWarmup()
    if _warmupCounter > 0 then
        _warmupCounter = _warmupCounter - 1
    end
end

function HeliSimService.isWarmedUp()
    return _warmupCounter <= 0
end

-------------------------------------------------------------------------------------
-- update() — one frame of horizontal flight orchestration
--
-- The core processing chain. No math — only coordination.
-- Returns results table for debug/telemetry.
-------------------------------------------------------------------------------------
function HeliSimService.update(vehicle, playerObj, keys, fpsMultiplier, heliType, curr_z, tempVector2)
    local toLuaNum = HeliUtil.toLuaNum
    local freeMode = vehicle:getModData().AutoBalance == true
    _flightAssistOff = freeMode

    -- 1. Init HeliOrientation from vehicle if not initialized
    if not HeliOrientation.isInitialized() then
        HeliOrientation.initFromVehicle(
            toLuaNum(vehicle:getAngleX()),
            toLuaNum(vehicle:getAngleY()),
            toLuaNum(vehicle:getAngleZ()))
    end

    -- 2. HeliInputProcessor: keys → rotation deltas
    local ax, ay, az, isRotating = HeliInputProcessor.computeRotationDeltas(
        keys, fpsMultiplier, heliType, playerObj, vehicle, tempVector2, freeMode)

    -- 3. Apply tilt + yaw to HeliOrientation
    HeliOrientation.applyTilt(ax, az)
    HeliOrientation.applyYaw(ay)

    -- 4. Yaw MPC: track intended heading, hard lock when not rotating
    local simYaw = HeliYawController.update(HeliOrientation.getYaw(), isRotating, ay)
    if not isRotating then
        HeliOrientation.setYaw(simYaw)
    end

    -- 5. setAngles output
    vehicle:setAngles(HeliOrientation.toEuler())

    -- 6. Read forward direction + body angles
    local fwdX, fwdZ = HeliOrientation.getForward()
    local angleZ = HeliOrientation.getBodyPitch()
    local angleX = HeliOrientation.getBodyRoll()

    -- 7. Wall pre-blocking: per-axis collision checks, zero blocked deviations
    local angle_90 = math.rad(90)
    local pitchDev = angleZ - angle_90
    local rollDev = angleX - angle_90
    local isBlockedHit = false

    local cosZ = math.cos(angleZ)
    local cosX = math.cos(angleX)
    local pitchBlocked = false
    local rollBlocked = false

    if math.abs(cosZ) > HeliConfig.DIRECTION_COS_THRESHOLD then
        if cosZ < 0 then
            pitchBlocked = HeliTerrainUtil.isBlocked(playerObj, "UP", vehicle, tempVector2)
        else
            pitchBlocked = HeliTerrainUtil.isBlocked(playerObj, "DOWN", vehicle, tempVector2)
        end
        if pitchBlocked then isBlockedHit = true end
    end
    if math.abs(cosX) > HeliConfig.DIRECTION_COS_THRESHOLD then
        if cosX < 0 then
            rollBlocked = HeliTerrainUtil.isBlocked(playerObj, "RIGHT", vehicle, tempVector2)
        else
            rollBlocked = HeliTerrainUtil.isBlocked(playerObj, "LEFT", vehicle, tempVector2)
        end
        if rollBlocked then isBlockedHit = true end
    end

    local effectivePitchDev = pitchBlocked and 0 or pitchDev
    local effectiveRollDev = rollBlocked and 0 or rollDev
    local totalTiltRad = math.sqrt(effectivePitchDev * effectivePitchDev + effectiveRollDev * effectiveRollDev)

    -- 8. HeliFilters pipeline: noiseFloor → thrustDecomposition → speedClamp → directionClamp
    local noiseFloor = HeliConfig.TILT_NOISE_FLOOR
    local maxHSpeed = HeliConfig.get("hspeed")

    local effectiveTilt = HeliFilters.applyNoiseFloor(totalTiltRad, noiseFloor)

    local totalVelX, totalVelZ, totalSpeed = HeliFilters.decomposeThrustDirection(
        effectivePitchDev, effectiveRollDev, totalTiltRad, fwdX, fwdZ, maxHSpeed, effectiveTilt)

    totalVelX, totalVelZ, totalSpeed = HeliFilters.clampSpeed(totalVelX, totalVelZ, maxHSpeed)

    -- Direction clamping: need movement angle from position delta
    local posX = toLuaNum(vehicle:getX())
    local posZ = toLuaNum(vehicle:getY())
    if _prevPosX and totalSpeed > 0.1 then
        local dx = posX - _prevPosX
        local dz = posZ - _prevPosZ
        local moveMag = math.sqrt(dx * dx + dz * dz)
        if moveMag > 0.08 then
            local moveAngle = math.atan2(dz, dx)
            totalVelX, totalVelZ = HeliFilters.clampThrustDirection(
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
        local _, _, svx, svz = HeliRawSim.getState()
        -- Read velocity for speed check
        local vx, _, vz = HeliVelocityAdapter.getVelocity(vehicle)
        local actualSpeed = math.sqrt(toLuaNum(vx) * toLuaNum(vx) + toLuaNum(vz) * toLuaNum(vz))
        if actualSpeed < HeliConfig.FA_OFF_DEADZONE then
            desiredHX, desiredHZ = 0, 0
            HeliSimService.initFlight(vehicle)
        else
            desiredHX, desiredHZ = svx, svz
        end
        hasHInput = true
        _hasHorizontalInput = true
    end

    -- 11. Select effective inertia
    if not _simInitialized then
        HeliSimService.initFlight(vehicle)
    end

    local fps = math.max(getAverageFPS(), HeliConfig.MIN_FPS)
    local dt = 1.0 / fps
    local baseBrake = HeliConfig.get("brake")
    local effectiveInertia = baseBrake * (hasHInput and HeliConfig.get("accel") or HeliConfig.get("decel"))

    -- 12. HeliRawSim.advance
    HeliRawSim.advance(desiredHX, desiredHZ, dt, effectiveInertia)

    -- 13. Heading re-anchor check (skip in FA-off — coast must survive turns)
    if not _flightAssistOff then
        if HeliYawController.checkHeadingReanchor(fps) then
            local actualX = toLuaNum(vehicle:getX())
            local actualZ = toLuaNum(vehicle:getY())
            HeliRawSim.snapPosition(actualX, actualZ)
            HeliErrorTracker.clearHistory()
        end
    end

    -- 14. Soft anchor: low speed + no input → blend sim toward actual
    local posDeltaSpeed = HeliVelocityAdapter.getPositionDeltaSpeed()
    local desiredMag = math.sqrt(desiredHX * desiredHX + desiredHZ * desiredHZ)
    if desiredMag < HeliConfig.SIM_SNAP_THRESHOLD and not hasHInput and posDeltaSpeed < HeliConfig.SIM_SNAP_THRESHOLD then
        HeliRawSim.blendToward(posX, posZ, HeliConfig.SIM_BLEND_RATE)
    end

    -- 15. Record in error tracker
    local simPosX, simPosZ, simVelX, simVelZ = HeliRawSim.getState()
    HeliErrorTracker.record(posX, posZ, simPosX, simPosZ)

    -- 16. Return results for HeliMove (debug, telemetry, state machine)
    local errX, errZ, errRateX, errRateZ = HeliErrorTracker.getPositionError()
    return {
        ax = ax, ay = ay, az = az,
        angleZ = angleZ, angleX = angleX,
        fwdX = fwdX, fwdZ = fwdZ,
        desiredVelX = desiredHX, desiredVelZ = desiredHZ,
        simVelX = simVelX, simVelZ = simVelZ,
        simPosX = simPosX, simPosZ = simPosZ,
        errX = errX, errZ = errZ,
        errRateX = errRateX, errRateZ = errRateZ,
        noHInput = noInput,
        isBlockedHit = isBlockedHit,
        hasHInput = hasHInput,
        freeMode = freeMode,
    }
end

-------------------------------------------------------------------------------------
-- applyCorrectionForces() — 0-frame delay correction path (OnTickEvenPaused)
-------------------------------------------------------------------------------------
function HeliSimService.applyCorrectionForces(vehicle)
    local toLuaNum = HeliUtil.toLuaNum
    local mass = toLuaNum(vehicle:getMass())

    local errX, errZ, errRateX, errRateZ = HeliErrorTracker.getPositionError()
    local errMag = math.sqrt(errX * errX + errZ * errZ)

    local vx, _, vz = HeliVelocityAdapter.getVelocity(vehicle)
    local hvx = toLuaNum(vx)
    local hvz = toLuaNum(vz)

    local fx, fz = HeliForceComputer.computeCorrectionForce(
        errX, errZ, errRateX, errRateZ, errMag,
        HeliConfig.get("pgain"), HeliConfig.get("dgain"),
        hvx, hvz, mass, HeliConfig.VEL_FORCE_FACTOR,
        HeliConfig.get("fstopgain"), _flightAssistOff,
        HeliConfig.FA_OFF_DEADZONE, HeliConfig.FA_OFF_MIN_DAMPING_SPEED)

    HeliForceAdapter.applyForceImmediate(vehicle, fx, 0, fz)
end

-------------------------------------------------------------------------------------
-- applyThrustForces() — vertical thrust (OnTick)
-------------------------------------------------------------------------------------
function HeliSimService.applyThrustForces(vehicle, desiredVelY, gravComp, savedVelY)
    local toLuaNum = HeliUtil.toLuaNum
    local mass = toLuaNum(vehicle:getMass())
    local Kp = HeliConfig.get("kp")
    local gravity = HeliConfig.get("gravity")

    local subSteps, physicsDelta = HeliForceAdapter.getSubStepsThisFrame()

    local fy = HeliForceComputer.computeThrustForce(
        desiredVelY, toLuaNum(savedVelY), mass, Kp, gravity,
        subSteps, physicsDelta, gravComp)

    if fy ~= 0 then
        HeliForceAdapter.applyForceImmediate(vehicle, 0, fy, 0)
    end
end
