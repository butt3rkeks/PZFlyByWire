--[[
    HEFContext — Framework context builder + contract

    Builds the HEFCtx table that the framework passes to flight engines
    every frame. This is the single source of truth for what engines receive.

    Engine authors: read the @class annotations below to know what ctx provides.
    The CTX_FIELDS table mirrors the annotations for runtime validation.
]]

HEFContext = {}

-------------------------------------------------------------------------------------
-- EmmyLua type annotations (IDE autocompletion for engine authors)
-------------------------------------------------------------------------------------

--- @class HEFKeys
--- @field up boolean
--- @field down boolean
--- @field left boolean
--- @field right boolean
--- @field w boolean
--- @field s boolean
--- @field a boolean
--- @field d boolean

--- @class HEFBlocked
--- @field up boolean Wall to the north (pitch forward blocked)
--- @field down boolean Wall to the south (pitch backward blocked)
--- @field left boolean Wall to the west (roll left blocked)
--- @field right boolean Wall to the east (roll right blocked)

--- @class HEFCtx
--- @field vehicle BaseVehicle The helicopter being flown
--- @field playerObj IsoPlayer The pilot
--- @field keys HEFKeys Keyboard state booleans
--- @field fpsMultiplier number TARGET_FPS / actualFPS (frame scaling)
--- @field fps number Current average FPS (clamped to MIN_FPS floor)
--- @field heliType string Helicopter type name (key into HeliList)
--- @field currentAltitude number Current altitude (z-levels)
--- @field groundLevelZ number Ground height under helicopter (z-levels)
--- @field scratchVector Vector3f Reusable scratch vector (PZ API)
--- @field posX number Vehicle horizontal position X (Lua number, PZ world)
--- @field posZ number Vehicle horizontal position Z (Lua number, PZ world Y)
--- @field velX number Smoothed horizontal velocity X (m/s)
--- @field velY number Raw vertical velocity Y (m/s)
--- @field velZ number Smoothed horizontal velocity Z (m/s)
--- @field mass number Vehicle mass (Lua number)
--- @field subSteps number Integer physics sub-steps this frame (0 at high FPS)
--- @field physicsDelta number Actual physics time this frame (seconds, may be fractional)
--- @field blocked HEFBlocked Wall collision state booleans
--- --- Vehicle state (read once, engines should prefer these over vehicle:get*) ---
--- @field fuelPercent number Remaining fuel percentage (0..100)
--- @field engineCondition number Engine part condition (0..100, or -1 if no engine part)
--- @field angleX number Vehicle Euler angle X (degrees, toLuaNum coerced)
--- @field angleY number Vehicle Euler angle Y (degrees, toLuaNum coerced)
--- @field angleZ number Vehicle Euler angle Z (degrees, toLuaNum coerced)
--- @field positionDeltaSpeed number Position-delta ground speed (m/s, immune to stale Bullet reads)
--- --- Output closures (engines should prefer these over direct adapter/game API calls) ---
--- @field applyForce fun(fx:number, fy:number, fz:number) Apply physics force (Bullet space, adapter-wrapped)
--- @field setAngles fun(x:number, y:number, z:number) Set vehicle Euler angles (degrees)
--- @field setPhysicsActive fun(active:boolean) Wake/sleep Bullet physics body

-------------------------------------------------------------------------------------
-- Runtime contract (mirrors EmmyLua above, used for validation)
-------------------------------------------------------------------------------------

local _types = {
    BaseVehicle = { label = "BaseVehicle", check = function(v) return type(v) == "userdata" end },
    IsoPlayer   = { label = "IsoPlayer",   check = function(v) return type(v) == "userdata" end },
    Vector3f    = { label = "Vector3f",    check = function(v) return type(v) == "userdata" end },
    number      = { label = "number",      check = function(v) return type(v) == "number" end },
    string      = { label = "string",      check = function(v) return type(v) == "string" end },
    bool_table  = { label = "table<string,boolean>", check = function(v) return type(v) == "table" end },
    key_table   = { label = "table<string,boolean>", check = function(v) return type(v) == "table" end },
}

HEFContext.CTX_FIELDS = {
    { name = "vehicle",       type = _types.BaseVehicle, desc = "The helicopter being flown" },
    { name = "playerObj",     type = _types.IsoPlayer,   desc = "The pilot" },
    { name = "keys",          type = _types.key_table,   desc = "Keyboard state booleans" },
    { name = "fpsMultiplier", type = _types.number,      desc = "TARGET_FPS / actualFPS (frame scaling)" },
    { name = "fps",           type = _types.number,      desc = "Current average FPS (clamped to MIN_FPS floor)" },
    { name = "heliType",      type = _types.string,      desc = "Helicopter type name (key into HeliList)" },
    { name = "currentAltitude", type = _types.number,     desc = "Current altitude (z-levels)" },
    { name = "groundLevelZ",  type = _types.number,      desc = "Ground height under helicopter (z-levels)" },
    { name = "scratchVector", type = _types.Vector3f,    desc = "Reusable scratch vector (PZ API)" },
    { name = "posX",          type = _types.number,      desc = "Vehicle horizontal position X (PZ world)" },
    { name = "posZ",          type = _types.number,      desc = "Vehicle horizontal position Z (PZ world Y)" },
    { name = "velX",          type = _types.number,      desc = "Smoothed horizontal velocity X (m/s)" },
    { name = "velY",          type = _types.number,      desc = "Raw vertical velocity Y (m/s)" },
    { name = "velZ",          type = _types.number,      desc = "Smoothed horizontal velocity Z (m/s)" },
    { name = "mass",          type = _types.number,      desc = "Vehicle mass (Lua number)" },
    { name = "subSteps",      type = _types.number,      desc = "Integer physics sub-steps this frame" },
    { name = "physicsDelta",  type = _types.number,      desc = "Actual physics time this frame (seconds)" },
    { name = "blocked",       type = _types.bool_table,  desc = "Wall collision state booleans" },
    { name = "fuelPercent",   type = _types.number,      desc = "Remaining fuel percentage (0..100)" },
    { name = "engineCondition", type = _types.number,    desc = "Engine part condition (0..100, -1 if absent)" },
    { name = "angleX",        type = _types.number,      desc = "Vehicle Euler angle X (degrees)" },
    { name = "angleY",        type = _types.number,      desc = "Vehicle Euler angle Y (degrees)" },
    { name = "angleZ",        type = _types.number,      desc = "Vehicle Euler angle Z (degrees)" },
    { name = "positionDeltaSpeed", type = _types.number, desc = "Position-delta ground speed (m/s)" },
}

-------------------------------------------------------------------------------------
-- Builder: constructs a complete HEFCtx from game state
--
-- Called once per frame by HeliMove. Reads keyboard, velocity, terrain, physics.
-- All per-frame side effects (cache invalidation, velocity adapter state,
-- physics accumulator) happen here so engines never need to call these APIs.
--
-- Allocation strategy: module-local tables are reused across frames to avoid
-- creating 7+ tables/closures per frame (keys, blocked, ctx, metatable, 3 closures).
-- Closures and metatable capture 'vehicle' — they are recreated only when the
-- vehicle reference changes (i.e., between flight sessions, not between frames).
-------------------------------------------------------------------------------------

-- Reusable tables (populated in-place each frame)
local _keys = {}
local _blocked = {}
local _ctx = {}

-- Vehicle-dependent closures and metatable (recreated on vehicle change)
local _cachedVehicle = nil
local _applyForceFn = nil
local _setAnglesFn = nil
local _setPhysicsActiveFn = nil
local _ctxMeta = nil

--- Rebuild closures and metatable when the vehicle reference changes.
--- Vehicle stays the same for an entire flight session — this runs once per
--- enter-vehicle, not once per frame.
--- @param vehicle BaseVehicle
local function _rebuildVehicleDeps(vehicle)
    _cachedVehicle = vehicle
    _applyForceFn = function(fx, fy, fz)
        HeliForceAdapter.applyForceImmediate(vehicle, fx, fy, fz)
    end
    _setAnglesFn = function(x, y, z)
        vehicle:setAngles(x, y, z)
    end
    _setPhysicsActiveFn = function(active)
        vehicle:setPhysicsActive(active)
    end
    -- Lazy fields: deferred until first access so framework side-effects
    -- (e.g., HeliAuxiliary.consumeGas) that run between build() and engine
    -- update() are reflected. Cached via rawset on first read.
    _ctxMeta = { __index = function(t, k)
        if k == "fuelPercent" then
            local v = vehicle:getRemainingFuelPercentage()
            rawset(t, k, v)
            return v
        elseif k == "engineCondition" then
            local enginePart = vehicle:getPartById("Engine")
            local v = enginePart and enginePart:getCondition() or -1
            rawset(t, k, v)
            return v
        end
    end }
end

--- Build the per-frame context table.
--- @param vehicle BaseVehicle
--- @param playerObj IsoPlayer
--- @param scratchVector Vector3f Reusable scratch vector
--- @return HEFCtx
function HEFContext.build(vehicle, playerObj, scratchVector)
    local toLuaNum = HeliUtil.toLuaNum

    -- Invalidate per-frame terrain cache before any isBlocked calls
    HeliTerrainUtil.invalidateBlockedCache()

    -- Rebuild closures/metatable if vehicle changed (once per flight session)
    if vehicle ~= _cachedVehicle then
        _rebuildVehicleDeps(vehicle)
    end

    -- Derived values
    local heliType = GetHeliType(vehicle)
    local fps = math.max(getAverageFPS(), HeliConfig.MIN_FPS)
    local fpsMultiplier = HeliConfig.TARGET_FPS / fps
    local currentAltitude = toLuaNum(vehicle:getWorldPos(0, 0, 0, scratchVector):z())
    local groundLevelZ = HeliTerrainUtil.getNowMaxZ(playerObj, currentAltitude)

    -- Vehicle state (read once, coerce once)
    local posX = toLuaNum(vehicle:getX())
    local posZ = toLuaNum(vehicle:getY())  -- PZ Y = world Z
    local mass = toLuaNum(vehicle:getMass())

    -- Vehicle angles (read once for engine init / orientation)
    local angleX = toLuaNum(vehicle:getAngleX())
    local angleY = toLuaNum(vehicle:getAngleY())
    local angleZ = toLuaNum(vehicle:getAngleZ())

    -- Keyboard state (repopulate reusable table)
    _keys.up    = isKeyDown(Keyboard.KEY_UP)
    _keys.down  = isKeyDown(Keyboard.KEY_DOWN)
    _keys.left  = isKeyDown(Keyboard.KEY_LEFT)
    _keys.right = isKeyDown(Keyboard.KEY_RIGHT)
    _keys.w     = isKeyDown(Keyboard.KEY_W)
    _keys.s     = isKeyDown(Keyboard.KEY_S)
    _keys.a     = isKeyDown(Keyboard.KEY_A)
    _keys.d     = isKeyDown(Keyboard.KEY_D)

    -- Velocity (one read per frame — adapter has stateful side effects)
    local rawVelX, rawVelY, rawVelZ = HeliVelocityAdapter.getVelocity(vehicle)

    -- Physics timing (one read per frame — accumulator advances)
    local subSteps, physicsDelta = HeliForceAdapter.getSubStepsThisFrame()

    -- Wall blocking (repopulate reusable table, uses per-frame cache)
    _blocked.up    = HeliTerrainUtil.isBlocked(playerObj, "UP", vehicle, scratchVector)
    _blocked.down  = HeliTerrainUtil.isBlocked(playerObj, "DOWN", vehicle, scratchVector)
    _blocked.left  = HeliTerrainUtil.isBlocked(playerObj, "LEFT", vehicle, scratchVector)
    _blocked.right = HeliTerrainUtil.isBlocked(playerObj, "RIGHT", vehicle, scratchVector)

    -- Position-delta speed (already computed as side-effect of getVelocity above)
    local positionDeltaSpeed = HeliVelocityAdapter.getPositionDeltaSpeed()

    -- Clear lazy-cached fields from previous frame (force fresh read from vehicle)
    _ctx.fuelPercent = nil
    _ctx.engineCondition = nil

    -- Repopulate reusable ctx table
    _ctx.vehicle = vehicle
    _ctx.playerObj = playerObj
    _ctx.keys = _keys
    _ctx.fpsMultiplier = fpsMultiplier
    _ctx.fps = fps
    _ctx.heliType = heliType
    _ctx.currentAltitude = currentAltitude
    _ctx.groundLevelZ = groundLevelZ
    _ctx.scratchVector = scratchVector
    _ctx.posX = posX
    _ctx.posZ = posZ
    _ctx.velX = toLuaNum(rawVelX)
    _ctx.velY = toLuaNum(rawVelY)
    _ctx.velZ = toLuaNum(rawVelZ)
    _ctx.mass = mass
    _ctx.subSteps = subSteps
    _ctx.physicsDelta = physicsDelta
    _ctx.blocked = _blocked
    _ctx.angleX = angleX
    _ctx.angleY = angleY
    _ctx.angleZ = angleZ
    _ctx.positionDeltaSpeed = positionDeltaSpeed
    _ctx.applyForce = _applyForceFn
    _ctx.setAngles = _setAnglesFn
    _ctx.setPhysicsActive = _setPhysicsActiveFn

    -- Metatable for lazy fields (same reference if vehicle unchanged)
    setmetatable(_ctx, _ctxMeta)

    return _ctx
end
