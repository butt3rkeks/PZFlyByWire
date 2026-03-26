--[[
    HeliMove — Orchestrator for UH1B Helicopter Physics Fix

    Thin controller tying all modules together. Registers event handlers,
    manages flight state transitions, delegates all computation to subsystems.

    Contains no physics math, no rotation math, no terrain scanning.
    All tuning constants come from HeliConfig.
]]

-------------------------------------------------------------------------------------
-- Globals (exported for HUD/UI systems in other mods)
-------------------------------------------------------------------------------------
local tempVector2 = Vector3f.new()
Heli_GlobalSpeed = 0
Heli_GlobalHeading = 0

-------------------------------------------------------------------------------------
-- Module lifecycle: init/cleanup registration
-- Modules call HeliMove.onFlightInit / HeliMove.onFlightCleanup at load time.
-- HeliMove iterates the lists on state transitions. New modules self-register
-- without touching HeliMove's handler bodies.
-------------------------------------------------------------------------------------
HeliMove = {}
local _initCallbacks = {}
local _cleanupCallbacks = {}

function HeliMove.onFlightInit(fn)
    _initCallbacks[#_initCallbacks + 1] = fn
end

function HeliMove.onFlightCleanup(fn)
    _cleanupCallbacks[#_cleanupCallbacks + 1] = fn
end

-------------------------------------------------------------------------------------
-- Wall damage rate limiter
-------------------------------------------------------------------------------------
local _wallDamageTick = HeliConfig.WALL_DAMAGE_INTERVAL

-------------------------------------------------------------------------------------
-- Dual-path: OnTickEvenPaused applies corrections from position error.
-- OnTick updates simulation model and applies vertical thrust.
-------------------------------------------------------------------------------------
local _dualPathActive = false

-------------------------------------------------------------------------------------
-- Explicit flight state
-- INACTIVE: not flying, or ground/engine-off (triggers re-init next frame)
-- WARMUP:   first 10 airborne frames (sim re-anchors, corrections disabled)
-- AIRBORNE: normal airborne flight (full force system active)
-------------------------------------------------------------------------------------
local STATE_INACTIVE = "inactive"
local STATE_WARMUP   = "warmup"
local STATE_AIRBORNE = "airborne"

local _flightState = STATE_INACTIVE


-------------------------------------------------------------------------------------
-- Validation
-------------------------------------------------------------------------------------
local function checkViald(playerObj, vehicle)
    if not playerObj then return false end
    if not vehicle then return false end
    if not GetHeliType(vehicle) then return false end
    local seat = vehicle:getSeat(playerObj)
    if seat ~= 0 then return false end
    return true
end

-------------------------------------------------------------------------------------
-- Read all keys once per frame into a table
-------------------------------------------------------------------------------------
local function readKeys()
    return {
        up    = isKeyDown(Keyboard.KEY_UP),
        down  = isKeyDown(Keyboard.KEY_DOWN),
        left  = isKeyDown(Keyboard.KEY_LEFT),
        right = isKeyDown(Keyboard.KEY_RIGHT),
        w     = isKeyDown(Keyboard.KEY_W),
        s     = isKeyDown(Keyboard.KEY_S),
        a     = isKeyDown(Keyboard.KEY_A),
        d     = isKeyDown(Keyboard.KEY_D),
    }
end

-------------------------------------------------------------------------------------
-- Main OnTick handler
-------------------------------------------------------------------------------------
local function helicopterMovementUpdate()
    local playerObj = getPlayer()
    if not playerObj then return end
    local vehicle = playerObj:getVehicle()
    if not checkViald(playerObj, vehicle) then
        _flightState = STATE_INACTIVE
        _dualPathActive = false
        return
    end

    local heliType = GetHeliType(vehicle)
    local fpsMultiplier = HeliConfig.TARGET_FPS / math.max(getAverageFPS(), HeliConfig.MIN_FPS)
    local curr_z = HeliUtil.toLuaNum(vehicle:getWorldPos(0, 0, 0, tempVector2):z())

    -- Read all keys once per frame
    local keys = readKeys()

    -- Prevent CarController speed regulation from fighting our forces.
    vehicle:setMaxSpeed(HeliConfig.MAX_SPEED_OVERRIDE)

    -- Invalidate per-frame caches
    HeliTerrainUtil.invalidateBlockedCache()

    -- Ghost mode
    HeliAuxiliary.updateGhostMode(playerObj, curr_z)

    -- Landing surface detection
    local nowMaxZ = HeliTerrainUtil.getNowMaxZ(playerObj, curr_z)

    -- Flight state initialization (first frame entering flight)
    if _flightState == STATE_INACTIVE then
        -- Core init (always first — clean slate before external callbacks)
        HeliSimService.resetFlightState()
        HeliSimService.initFlight(vehicle)
        -- External module callbacks (run against clean state)
        for _, fn in ipairs(_initCallbacks) do fn(vehicle) end
        _flightState = STATE_WARMUP
    end

    -- Warmup: re-anchor sim to actual position each frame until physics stabilizes.
    if _flightState == STATE_WARMUP then
        HeliSimService.tickWarmup()
        HeliSimService.initFlight(vehicle)
        _dualPathActive = false
        if HeliSimService.isWarmedUp() then
            _flightState = STATE_AIRBORNE
        end
    end

    -- Read velocity ONCE per OnTick. Physics doesn't step during OnTick,
    -- so all reads within this handler return the same values.
    local rawVelX, rawVelY, rawVelZ = HeliVelocityAdapter.getVelocity(vehicle)
    local velX = HeliUtil.toLuaNum(rawVelX)
    local velY = HeliUtil.toLuaNum(rawVelY)
    local velZ = HeliUtil.toLuaNum(rawVelZ)

    -- === ENGINE OFF PATH ===
    if not vehicle:isEngineRunning() then
        _flightState = STATE_INACTIVE
        _dualPathActive = false
        if curr_z > nowMaxZ then
            local fallSpeed = HeliConfig.get("fall")
            HeliSimService.applyThrustForces(vehicle, -fallSpeed, false, velY)
            if HeliDebug.logEnabled then
                HeliDebug.logEngineOff(curr_z, fallSpeed, velX, velY, velZ)
            end
        end
        return
    end

    -- NightLight
    HeliAuxiliary.updateNightLight(playerObj, vehicle, nowMaxZ)

    -- Gas consumption
    HeliAuxiliary.consumeGas(vehicle, curr_z, fpsMultiplier, heliType)

    -- Airborne threshold: relative to actual ground surface
    local isAirborne = (curr_z > nowMaxZ + HeliConfig.AIRBORNE_MARGIN)

    -- Results from HeliSimService.update (populated in airborne path)
    local r = nil

    -- === AIRBORNE PATH ===
    if isAirborne then
        -- Don't overwrite _flightState during warmup — warmup completion (above)
        -- transitions to AIRBORNE. Overwriting here would skip warmup ticking.
        if _flightState ~= STATE_WARMUP then
            _flightState = STATE_AIRBORNE
        end

        -- Horizontal flight: rotation + filters + simulation (all via facade)
        r = HeliSimService.update(vehicle, playerObj, keys, fpsMultiplier, heliType, curr_z, tempVector2)

        -- Wall damage (rate-limited)
        if r.isBlockedHit then
            if _wallDamageTick > 0 then
                _wallDamageTick = _wallDamageTick - 1
            else
                _wallDamageTick = math.floor(HeliConfig.WALL_DAMAGE_INTERVAL * getAverageFPS() / 60)
                HeliAuxiliary.applyWallDamage(vehicle)
            end
        end

        -- Update global telemetry
        Heli_GlobalSpeed = (r.ax + r.ay + r.az) * fpsMultiplier * HeliConfig.TELEMETRY_SPEED_FACTOR
        Heli_GlobalHeading = vehicle:getAngleZ()

    -- === GROUND PATH ===
    else
        _flightState = STATE_INACTIVE
        local groundVelMag = math.abs(velX) + math.abs(velY) + math.abs(velZ)

        if keys.w and vehicle:getRemainingFuelPercentage() > 0 then
            vehicle:setPhysicsActive(true)
            local mass = HeliUtil.toLuaNum(vehicle:getMass())
            local ascendSpeed = HeliConfig.get("ascend")
            local gravity = HeliConfig.get("gravity")
            local subSteps = HeliForceAdapter.getSubStepsThisFrame()
            if subSteps > 0 then
                local Kp = HeliConfig.get("kp")
                local liftFy = Kp * (ascendSpeed - velY) * mass * subSteps + mass * gravity * subSteps
                HeliForceAdapter.applyForceImmediate(vehicle,
                    -velX * mass * HeliConfig.GROUND_VELOCITY_KILL,
                    liftFy,
                    -velZ * mass * HeliConfig.GROUND_VELOCITY_KILL)
            end
        elseif groundVelMag > HeliConfig.GROUND_VELOCITY_THRESHOLD then
            local mass = HeliUtil.toLuaNum(vehicle:getMass())
            HeliForceAdapter.applyForceImmediate(vehicle,
                -velX * mass * HeliConfig.GROUND_VELOCITY_KILL,
                -velY * mass * HeliConfig.GROUND_VELOCITY_KILL,
                -velZ * mass * HeliConfig.GROUND_VELOCITY_KILL)
        end

        vehicle:setSpeedKmHour(0)
        _dualPathActive = false
        return
    end

    -- === BELOW HERE: AIRBORNE ONLY ===

    -- === VERTICAL TARGET ===
    local freeMode = r.freeMode
    local targetVelY, gravComp, vBraking, engineDead = HeliFlightModel.computeVerticalTarget(
        vehicle, curr_z, freeMode, keys)

    -- Landing zone: smooth descent near ground to prevent oscillation.
    if targetVelY < 0 and curr_z < nowMaxZ + HeliConfig.LANDING_ZONE_HEIGHT then
        local landingFactor = math.max((curr_z - nowMaxZ) / HeliConfig.LANDING_ZONE_HEIGHT, 0)
        landingFactor = math.max(landingFactor, HeliConfig.LANDING_MIN_SPEED_FACTOR)
        targetVelY = targetVelY * landingFactor
    end

    -- Engine dead: shut off engine
    if engineDead and vehicle:isEngineRunning() then
        ISVehicleMenu.onShutOff(playerObj)
    end

    -- DUAL-PATH FORCE APPLICATION
    local errX, errZ = r.errX, r.errZ
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local hSpeedActual = math.sqrt(velX * velX + velZ * velZ)
    _dualPathActive = HeliSimService.isWarmedUp() and
        (r.hasHInput or errMag > HeliConfig.DUAL_PATH_ERROR_THRESHOLD or hSpeedActual > HeliConfig.DUAL_PATH_SPEED_THRESHOLD)

    -- Vertical thrust
    HeliSimService.applyThrustForces(vehicle, targetVelY, gravComp, velY)

    -- Smooth the speedometer display using simulated velocity.
    local simSpeed = math.sqrt(r.simVelX * r.simVelX + r.simVelZ * r.simVelZ) * 3.6  -- m/s → km/h
    vehicle:setSpeedKmHour(simSpeed)

    -- === KEY STATE (shared by debug + recorder) ===
    local keyStr = nil
    if HeliDebug.logEnabled or HeliDebug.snapRequested or HeliDebug.isRecording() then
        keyStr = ""
        if keys.up then keyStr = keyStr .. "U" end
        if keys.down then keyStr = keyStr .. "D" end
        if keys.left then keyStr = keyStr .. "L" end
        if keys.right then keyStr = keyStr .. "R" end
        if keys.w then keyStr = keyStr .. "W" end
        if keys.s then keyStr = keyStr .. "S" end
        if keys.a then keyStr = keyStr .. "a" end
        if keys.d then keyStr = keyStr .. "d" end
        if keyStr == "" then keyStr = "-" end
    end

    -- === DEBUG LOGGING ===
    if HeliDebug.logEnabled or HeliDebug.snapRequested then
        local fps = getAverageFPS()
        local subSteps = 1.0 / (math.max(fps, 10) * 0.01)

        HeliDebug.captureState({
            state = _flightState,
            curr_z = curr_z,
            nowMaxZ = nowMaxZ,
            desiredVelX = r.desiredVelX,
            desiredVelZ = r.desiredVelZ,
            simVelX = r.simVelX,
            simVelZ = r.simVelZ,
            posErrorX = errX,
            posErrorZ = errZ,
            posErrorRateX = r.errRateX,
            posErrorRateZ = r.errRateZ,
            savedVelY = velY,
            gravComp = gravComp,
            engineDead = engineDead,
            noHInput = r.noHInput,
            freeMode = freeMode,
            blocked = r.isBlockedHit,
            fps = fps,
            subSteps = subSteps,
            mass = vehicle:getMass(),
            keys = keyStr,
        })

        HeliDebug.periodicLog(_flightState, curr_z, r.desiredVelX, r.desiredVelZ,
            r.simVelX, r.simVelZ, errX, errZ, velY, keyStr)

        if HeliDebug.snapRequested then
            HeliDebug.snapRequested = false
        end
    end

    -- === FLIGHT DATA RECORDER ===
    if HeliDebug.isRecording() then
        local simPosX, simPosZ, simVelX, simVelZ = HeliSimService.getSimState()
        local recErrX, recErrZ = HeliSimService.getPositionError()
        HeliDebug.writeFlightFrame({
            ms       = getTimestampMs(),
            state    = _flightState,
            fps      = getAverageFPS(),
            aX       = HeliUtil.toLuaNum(vehicle:getX()),
            aZ       = HeliUtil.toLuaNum(vehicle:getY()),
            alt      = curr_z,
            vX       = velX,
            vY       = velY,
            vZ       = velZ,
            sX       = simPosX,
            sZ       = simPosZ,
            svX      = simVelX,
            svZ      = simVelZ,
            eX       = recErrX,
            eZ       = recErrZ,
            dvX      = r.desiredVelX,
            dvZ      = r.desiredVelZ,
            dvY      = targetVelY,
            sub      = HeliForceAdapter.getLastSubSteps(),
            tilt     = HeliSimService.hasTiltInput(),
            faOff    = HeliSimService.isFlightAssistOff(),
            yawSim   = HeliSimService.getIntendedYaw(),
            yawAct   = HeliUtil.toLuaNum(vehicle:getAngleY()),
            gravComp = gravComp,
            dualPath = _dualPathActive,
            keys     = keyStr or "-",
        })
    end

end

-------------------------------------------------------------------------------------
-- Exit vehicle handler
-------------------------------------------------------------------------------------
local function helicopterExit(player)
    local vehicle = player:getVehicle()
    if vehicle == nil then return end
    local seat = vehicle:getSeat(player)
    if seat ~= 0 then return end
    if not GetHeliType(vehicle) then return end

    -- Stop flight recorder if active (before cleanup, so last frame is captured)
    if HeliDebug.isRecording() then
        HeliDebug.stopRecording()
    end
    -- Core cleanup (always runs)
    HeliAuxiliary.cleanup(player)
    -- External module callbacks
    for _, fn in ipairs(_cleanupCallbacks) do fn(player, vehicle) end
    _dualPathActive = false
    _flightState = STATE_INACTIVE
end

-------------------------------------------------------------------------------------
-- Black zone warning (world boundary check)
-------------------------------------------------------------------------------------
local function checkBlackZone()
    local player = getPlayer()
    if not player then return end
    local vehicle = player:getVehicle()
    if not vehicle or not GetHeliType(vehicle) then return end

    local playerX, playerY = player:getX(), player:getY()
    local dist = HeliConfig.BLACK_ZONE_CHECK_DISTANCE
    local directions = {
        { x = playerX + dist, y = playerY },
        { x = playerX - dist, y = playerY },
        { x = playerX, y = playerY + dist },
        { x = playerX, y = playerY - dist },
    }
    for _, dir in ipairs(directions) do
        local square = getCell():getGridSquare(dir.x, dir.y, 0)
        if not square then
            player:Say(getText("IGUI_BlackZoneWarn"))
            vehicle:playSound("JetAlarm")
            return
        end
    end
end

-------------------------------------------------------------------------------------
-- OnTickEvenPaused: Apply CORRECTION forces (0-frame delay path).
-------------------------------------------------------------------------------------
local function helicopterCorrectionUpdate()
    if not _dualPathActive then return end

    if GameClient and not GameClient.client then
        local sc = UIManager and UIManager.getSpeedControls and UIManager.getSpeedControls()
        if sc and sc:getCurrentGameSpeed() == 0 then return end
    end

    local playerObj = getPlayer()
    if not playerObj then return end
    local vehicle = playerObj:getVehicle()
    if not vehicle then return end
    if not GetHeliType(vehicle) then return end

    HeliSimService.applyCorrectionForces(vehicle)
end

-------------------------------------------------------------------------------------
-- Event registration
-------------------------------------------------------------------------------------
Events.OnTickEvenPaused.Add(helicopterCorrectionUpdate)
Events.OnTick.Add(helicopterMovementUpdate)
Events.OnExitVehicle.Add(helicopterExit)
Events.EveryOneMinute.Add(checkBlackZone)
