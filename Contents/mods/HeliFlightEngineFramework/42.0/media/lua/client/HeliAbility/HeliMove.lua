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
local tempVector2 = Vector3f.new()
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
local _wallDamageTick = HeliConfig.GetWalldmg()

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
    local ctx = HEFContext.build(vehicle, playerObj, tempVector2)
    local curr_z = ctx.curr_z
    local nowMaxZ = ctx.nowMaxZ
    local keys = ctx.keys
    local velX = ctx.velX
    local velY = ctx.velY
    local velZ = ctx.velZ
    local fpsMultiplier = ctx.fpsMultiplier
    local heliType = ctx.heliType

    -- Ghost mode
    HeliAuxiliary.updateGhostMode(playerObj, curr_z)

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
        if curr_z > nowMaxZ then
            local fallSpeed = HeliConfig.GetFall()
            local Kp = HeliConfig.GetFallgain()
            if ctx.subSteps > 0 then
                local fy = Kp * (-fallSpeed - velY) * ctx.mass * ctx.subSteps
                if fy ~= 0 then
                    HeliForceAdapter.applyForceImmediate(vehicle, 0, fy, 0)
                end
            end
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

    local r = nil

    -- === AIRBORNE PATH ===
    if isAirborne then
        if _flightState ~= STATE_WARMUP then
            _flightState = STATE_AIRBORNE
        end

        r = HeliSimService.update(ctx)

        -- Wall damage (rate-limited)
        if r.isBlockedHit then
            if _wallDamageTick > 0 then
                _wallDamageTick = _wallDamageTick - 1
            else
                _wallDamageTick = math.floor(HeliConfig.GetWalldmg() * getAverageFPS() / 60)
                HeliAuxiliary.applyWallDamage(vehicle)
            end
        end

        -- Update global telemetry
        Heli_GlobalSpeed = r.telemetrySpeed
        Heli_GlobalHeading = vehicle:getAngleZ()

    -- === GROUND PATH ===
    else
        _flightState = STATE_INACTIVE

        local gr = HeliSimService.updateGround(ctx)

        vehicle:setSpeedKmHour(gr.displaySpeed)
        _dualPathActive = false
        return
    end

    -- === BELOW HERE: AIRBORNE ONLY ===

    -- Engine dead: shut off engine (gameplay side-effect, framework responsibility)
    if r.engineDead and vehicle:isEngineRunning() then
        ISVehicleMenu.onShutOff(playerObj)
    end

    -- Dual-path activation (from engine results)
    _dualPathActive = r.dualPathActive

    -- Display speed
    vehicle:setSpeedKmHour(r.displaySpeed)

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
            posErrorX = r.errX,
            posErrorZ = r.errZ,
            posErrorRateX = r.errRateX,
            posErrorRateZ = r.errRateZ,
            savedVelY = velY,
            gravComp = r.gravComp,
            engineDead = r.engineDead,
            noHInput = r.noHInput,
            freeMode = r.freeMode,
            blocked = r.isBlockedHit,
            fps = fps,
            subSteps = subSteps,
            mass = vehicle:getMass(),
            keys = keyStr,
        })

        HeliDebug.periodicLog(_flightState, curr_z, r.desiredVelX or 0, r.desiredVelZ or 0,
            r.simVelX or 0, r.simVelZ or 0, r.errX or 0, r.errZ or 0, velY, keyStr)

        if HeliDebug.snapRequested then
            HeliDebug.snapRequested = false
        end
    end

    -- === FLIGHT DATA RECORDER ===
    -- Read raw Bullet velocity directly (don't call getVelocity() again —
    -- it has stateful side effects that corrupt stale-read detection).
    if HeliDebug.isRecording() then
        local dbg = HeliSimService.getDebugState()
        local recVel = vehicle:getLinearVelocity(Vector3f.new())
        HeliDebug.writeFlightFrame({
            ms       = getTimestampMs(),
            state    = _flightState,
            fps      = getAverageFPS(),
            aX       = HeliUtil.toLuaNum(vehicle:getX()),
            aZ       = HeliUtil.toLuaNum(vehicle:getY()),
            alt      = curr_z,
            vX       = HeliUtil.toLuaNum(recVel:x()),
            vY       = HeliUtil.toLuaNum(recVel:y()),
            vZ       = HeliUtil.toLuaNum(recVel:z()),
            sX       = dbg.simPosX or 0,
            sZ       = dbg.simPosZ or 0,
            svX      = dbg.simVelX or 0,
            svZ      = dbg.simVelZ or 0,
            eX       = dbg.errX or 0,
            eZ       = dbg.errZ or 0,
            dvX      = r.desiredVelX or 0,
            dvZ      = r.desiredVelZ or 0,
            dvY      = r.targetVelY or 0,
            sub      = HeliForceAdapter.getLastSubSteps(),
            tilt     = r.hasHInput,
            faOff    = r.freeMode,
            yawSim   = HeliSimService.getIntendedYaw() or 0,
            yawAct   = HeliUtil.toLuaNum(vehicle:getAngleY()),
            gravComp = r.gravComp,
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

    if HeliDebug.isRecording() then
        HeliDebug.stopRecording()
    end
    HeliAuxiliary.cleanup(player)
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
