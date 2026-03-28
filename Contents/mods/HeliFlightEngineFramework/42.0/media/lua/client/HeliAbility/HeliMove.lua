--[[
    HeliMove — Orchestrator for Helicopter Flight Engine Framework

    Thin controller tying framework + active engine together. Registers event
    handlers, manages flight state transitions, delegates all computation to
    HeliSimService (which dispatches to the active engine).

    Contains no physics math, no rotation math, no terrain scanning.
    All tuning constants come from HeliConfig.
]]

-------------------------------------------------------------------------------------
-- Globals (exported for HUD/UI systems in other mods)
-------------------------------------------------------------------------------------
local scratchVector = Vector3f.new()
Heli_GlobalSpeed = 0
Heli_GlobalHeading = 0

-------------------------------------------------------------------------------------
-- Module lifecycle: init/cleanup registration
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
local _wallDamageTick = HeliConfig.GetWallDamageInterval()

-------------------------------------------------------------------------------------
-- Dual-path: OnTickEvenPaused applies corrections from position error.
-------------------------------------------------------------------------------------
local _dualPathActive = false

-------------------------------------------------------------------------------------
-- Flight state machine
-------------------------------------------------------------------------------------
local STATE_INACTIVE = "inactive"
local STATE_WARMUP   = "warmup"
local STATE_AIRBORNE = "airborne"

local _flightState = STATE_INACTIVE

-------------------------------------------------------------------------------------
-- Validation
-------------------------------------------------------------------------------------
local function checkValid(playerObj, vehicle)
    if not playerObj then return false end
    if not vehicle then return false end
    if not GetHeliType(vehicle) then return false end
    local seat = vehicle:getSeat(playerObj)
    if seat ~= 0 then return false end
    return true
end

-------------------------------------------------------------------------------------
-- Main OnTick handler
-------------------------------------------------------------------------------------
local function helicopterMovementUpdate()
    local playerObj = getPlayer()
    if not playerObj then return end
    local vehicle = playerObj:getVehicle()
    if not checkValid(playerObj, vehicle) then
        if _flightState ~= STATE_INACTIVE then
            -- Was flying, now invalid (seat switch, exit, etc.) — clear ghost mode
            HeliAuxiliary.cleanup(playerObj)
        end
        _flightState = STATE_INACTIVE
        _dualPathActive = false
        return
    end

    -- Prevent CarController speed regulation from fighting our forces.
    vehicle:setMaxSpeed(HeliConfig.MAX_SPEED_OVERRIDE)

    -- Flight state initialization (first frame entering flight)
    -- Must run BEFORE HEFContext.build() so velocity adapter is reset before first read.
    if _flightState == STATE_INACTIVE then
        HeliSimService.resetFlightState()
        HeliSimService.initFlight(vehicle)
        for _, fn in ipairs(_initCallbacks) do fn(vehicle) end
        _flightState = STATE_WARMUP
    end

    -- Build per-frame context (keys, velocity, terrain, wall blocking)
    -- Runs after reset so HeliVelocityAdapter starts clean on first flight frame.
    --- @type HEFCtx
    local ctx = HEFContext.build(vehicle, playerObj, scratchVector)
    local currentAltitude = ctx.currentAltitude
    local groundLevelZ = ctx.groundLevelZ
    local keys = ctx.keys
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ
    local fpsMultiplier = ctx.fpsMultiplier
    local heliType = ctx.heliType

    -- Ghost mode + Z-level for all occupants
    HeliAuxiliary.updateGhostMode(playerObj, currentAltitude, vehicle)

    -- Warmup: re-anchor sim to actual position each frame until physics stabilizes.
    if _flightState == STATE_WARMUP then
        HeliSimService.tickWarmup()
        HeliSimService.initFlight(vehicle)
        _dualPathActive = false
        if HeliSimService.isWarmedUp() then
            _flightState = STATE_AIRBORNE
        end
    end

    -- === ENGINE OFF PATH ===
    -- Framework-level fall (engine-agnostic): simple PD toward fall speed, no gravity comp.
    if not vehicle:isEngineRunning() then
        _flightState = STATE_INACTIVE
        _dualPathActive = false
        if currentAltitude > groundLevelZ then
            local fallSpeed = HeliConfig.GetEngineOffFallSpeed()
            local fallGain = HeliConfig.GetFallPdGain()
            if ctx.subSteps > 0 then
                local fallForceY = fallGain * (-fallSpeed - velY) * ctx.mass
                if fallForceY ~= 0 then
                    HeliForceAdapter.applyForceImmediate(vehicle, 0, fallForceY, 0)
                end
            end
            if HeliDebug.logEnabled then
                HeliDebug.logEngineOff(currentAltitude, fallSpeed, velX, velY, velZ)
            end
        end
        return
    end

    -- NightLight
    HeliAuxiliary.updateNightLight(playerObj, vehicle, groundLevelZ)

    -- Gas consumption
    HeliAuxiliary.consumeGas(vehicle, currentAltitude, fpsMultiplier, heliType)

    -- Airborne threshold: relative to actual ground surface
    local isAirborne = (currentAltitude > groundLevelZ + HeliConfig.AIRBORNE_MARGIN)

    local flightResult = nil

    -- === AIRBORNE PATH ===
    if isAirborne then
        if _flightState ~= STATE_WARMUP then
            _flightState = STATE_AIRBORNE
        end

        flightResult = HeliSimService.update(ctx)

        -- Wall damage (rate-limited)
        if flightResult.isBlockedHit then
            if _wallDamageTick > 0 then
                _wallDamageTick = _wallDamageTick - 1
            else
                _wallDamageTick = math.floor(HeliConfig.GetWallDamageInterval() * getAverageFPS() / 60)
                HeliAuxiliary.applyWallDamage(vehicle)
            end
        end

        -- Update global telemetry
        Heli_GlobalSpeed = flightResult.telemetrySpeed
        Heli_GlobalHeading = vehicle:getAngleZ()

    -- === GROUND PATH ===
    else
        local groundResult = HeliSimService.updateGround(ctx)

        -- Engine ground handlers can request flight state preservation across
        -- the ground phase (e.g. FBW keeps sim/orientation alive in transition zone).
        if not groundResult.keepFlightState then
            _flightState = STATE_INACTIVE
        end

        vehicle:setSpeedKmHour(groundResult.displaySpeed)
        _dualPathActive = false
        return
    end

    -- === BELOW HERE: AIRBORNE ONLY ===

    -- Engine dead: shut off engine (gameplay side-effect, framework responsibility)
    if flightResult.engineDead and vehicle:isEngineRunning() then
        ISVehicleMenu.onShutOff(playerObj)
    end

    -- Dual-path activation (from engine results)
    _dualPathActive = flightResult.dualPathActive

    -- Display speed
    vehicle:setSpeedKmHour(flightResult.displaySpeed)

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
            currentAltitude = currentAltitude,
            groundLevelZ = groundLevelZ,
            desiredVelX = flightResult.desiredVelX,
            desiredVelZ = flightResult.desiredVelZ,
            simVelX = flightResult.simVelX,
            simVelZ = flightResult.simVelZ,
            posErrorX = flightResult.errX,
            posErrorZ = flightResult.errZ,
            posErrorRateX = flightResult.errRateX,
            posErrorRateZ = flightResult.errRateZ,
            savedVelY = velY,
            gravComp = flightResult.gravComp,
            engineDead = flightResult.engineDead,
            noHInput = flightResult.noHInput,
            freeMode = flightResult.freeMode,
            blocked = flightResult.isBlockedHit,
            fps = fps,
            subSteps = subSteps,
            mass = vehicle:getMass(),
            keys = keyStr,
        })

        HeliDebug.periodicLog(_flightState, currentAltitude, flightResult.desiredVelX or 0, flightResult.desiredVelZ or 0,
            flightResult.simVelX or 0, flightResult.simVelZ or 0, flightResult.errX or 0, flightResult.errZ or 0, velY, keyStr)

        if HeliDebug.snapRequested then
            HeliDebug.snapRequested = false
        end
    end

    -- === FLIGHT DATA RECORDER ===
    -- Read raw Bullet velocity directly (don't call getVelocity() again —
    -- it has stateful side effects that corrupt stale-read detection).
    if HeliDebug.isRecording() then
        local engineDebugState = HeliSimService.getDebugState()
        local recordingVelocity = vehicle:getLinearVelocity(Vector3f.new())
        HeliDebug.writeFlightFrame({
            ms          = getTimestampMs(),
            state       = _flightState,
            fps         = getAverageFPS(),
            actualX     = HeliUtil.toLuaNum(vehicle:getX()),
            actualZ     = HeliUtil.toLuaNum(vehicle:getY()),
            alt         = currentAltitude,
            velocityX   = HeliUtil.toLuaNum(recordingVelocity:x()),
            velocityY   = HeliUtil.toLuaNum(recordingVelocity:y()),
            velocityZ   = HeliUtil.toLuaNum(recordingVelocity:z()),
            desiredVelX = flightResult.desiredVelX or 0,
            desiredVelZ = flightResult.desiredVelZ or 0,
            desiredVelY = flightResult.targetVelY or 0,
            subSteps    = HeliForceAdapter.getLastSubSteps(),
            tilt        = flightResult.hasHInput,
            flightAssistOff = flightResult.freeMode,
            yawSimulated = HeliSimService.getIntendedYaw() or 0,
            yawActual   = HeliUtil.toLuaNum(vehicle:getAngleY()),
            gravComp    = flightResult.gravComp,
            dualPath    = _dualPathActive,
            keys        = keyStr or "-",
        }, engineDebugState)
    end

end

-------------------------------------------------------------------------------------
-- Exit vehicle handler
-------------------------------------------------------------------------------------
local function helicopterExit(player)
    local vehicle = player:getVehicle()
    if vehicle == nil then return end
    if not GetHeliType(vehicle) then return end

    -- Any occupant exiting: clear their ghost mode + Z
    if player:isGhostMode() and not player:isGodMod() then
        player:setGhostMode(false)
    end
    player:setZ(0)

    -- Pilot-only cleanup: flight state, debug, callbacks
    local seat = vehicle:getSeat(player)
    if seat == 0 then
        if HeliDebug.isRecording() then
            HeliDebug.stopRecording()
        end
        HeliAuxiliary.cleanup(player)
        for _, fn in ipairs(_cleanupCallbacks) do fn(player, vehicle) end
        _dualPathActive = false
        _flightState = STATE_INACTIVE
    end
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

    --- @type HEFCorrectionCtx
    local cctx = HEFCorrectionCtx.build(vehicle)
    HeliSimService.applyCorrectionForces(cctx)
end

-------------------------------------------------------------------------------------
-- Event registration
-------------------------------------------------------------------------------------
Events.OnTickEvenPaused.Add(helicopterCorrectionUpdate)
Events.OnTick.Add(helicopterMovementUpdate)
Events.OnExitVehicle.Add(helicopterExit)
Events.EveryOneMinute.Add(checkBlackZone)
