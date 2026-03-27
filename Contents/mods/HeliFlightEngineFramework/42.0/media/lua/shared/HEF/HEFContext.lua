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
--- @field curr_z number Current altitude (z-levels)
--- @field nowMaxZ number Ground height under helicopter (z-levels)
--- @field tempVector2 Vector3f Reusable scratch vector (PZ API)
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
    { name = "curr_z",        type = _types.number,      desc = "Current altitude (z-levels)" },
    { name = "nowMaxZ",       type = _types.number,      desc = "Ground height under helicopter (z-levels)" },
    { name = "tempVector2",   type = _types.Vector3f,    desc = "Reusable scratch vector (PZ API)" },
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
-------------------------------------------------------------------------------------

--- Build the per-frame context table.
--- @param vehicle BaseVehicle
--- @param playerObj IsoPlayer
--- @param tempVector2 Vector3f Reusable scratch vector
--- @return HEFCtx
function HEFContext.build(vehicle, playerObj, tempVector2)
    local toLuaNum = HeliUtil.toLuaNum

    -- Invalidate per-frame terrain cache before any isBlocked calls
    HeliTerrainUtil.invalidateBlockedCache()

    -- Derived values
    local heliType = GetHeliType(vehicle)
    local fps = math.max(getAverageFPS(), HeliConfig.MIN_FPS)
    local fpsMultiplier = HeliConfig.TARGET_FPS / fps
    local curr_z = toLuaNum(vehicle:getWorldPos(0, 0, 0, tempVector2):z())
    local nowMaxZ = HeliTerrainUtil.getNowMaxZ(playerObj, curr_z)

    -- Vehicle state (read once, coerce once)
    local posX = toLuaNum(vehicle:getX())
    local posZ = toLuaNum(vehicle:getY())  -- PZ Y = world Z
    local mass = toLuaNum(vehicle:getMass())

    -- Vehicle angles (read once for engine init / orientation)
    local angleX = toLuaNum(vehicle:getAngleX())
    local angleY = toLuaNum(vehicle:getAngleY())
    local angleZ = toLuaNum(vehicle:getAngleZ())

    -- Keyboard state
    local keys = {
        up    = isKeyDown(Keyboard.KEY_UP),
        down  = isKeyDown(Keyboard.KEY_DOWN),
        left  = isKeyDown(Keyboard.KEY_LEFT),
        right = isKeyDown(Keyboard.KEY_RIGHT),
        w     = isKeyDown(Keyboard.KEY_W),
        s     = isKeyDown(Keyboard.KEY_S),
        a     = isKeyDown(Keyboard.KEY_A),
        d     = isKeyDown(Keyboard.KEY_D),
    }

    -- Velocity (one read per frame — adapter has stateful side effects)
    local rawVelX, rawVelY, rawVelZ = HeliVelocityAdapter.getVelocity(vehicle)

    -- Physics timing (one read per frame — accumulator advances)
    local subSteps, physicsDelta = HeliForceAdapter.getSubStepsThisFrame()

    -- Wall blocking (uses per-frame cache)
    local blocked = {
        up    = HeliTerrainUtil.isBlocked(playerObj, "UP", vehicle, tempVector2),
        down  = HeliTerrainUtil.isBlocked(playerObj, "DOWN", vehicle, tempVector2),
        left  = HeliTerrainUtil.isBlocked(playerObj, "LEFT", vehicle, tempVector2),
        right = HeliTerrainUtil.isBlocked(playerObj, "RIGHT", vehicle, tempVector2),
    }

    -- Position-delta speed (already computed as side-effect of getVelocity above)
    local positionDeltaSpeed = HeliVelocityAdapter.getPositionDeltaSpeed()

    local ctx = {
        vehicle = vehicle,
        playerObj = playerObj,
        keys = keys,
        fpsMultiplier = fpsMultiplier,
        fps = fps,
        heliType = heliType,
        curr_z = curr_z,
        nowMaxZ = nowMaxZ,
        tempVector2 = tempVector2,
        posX = posX,
        posZ = posZ,
        velX = toLuaNum(rawVelX),
        velY = toLuaNum(rawVelY),
        velZ = toLuaNum(rawVelZ),
        mass = mass,
        subSteps = subSteps,
        physicsDelta = physicsDelta,
        blocked = blocked,
        -- Vehicle state (eagerly read)
        angleX = angleX,
        angleY = angleY,
        angleZ = angleZ,
        positionDeltaSpeed = positionDeltaSpeed,
        -- Output closures
        applyForce = function(fx, fy, fz)
            HeliForceAdapter.applyForceImmediate(vehicle, fx, fy, fz)
        end,
        setAngles = function(x, y, z)
            vehicle:setAngles(x, y, z)
        end,
        setPhysicsActive = function(active)
            vehicle:setPhysicsActive(active)
        end,
    }

    -- Lazy fields: deferred until first access so framework side-effects
    -- (e.g., HeliAuxiliary.consumeGas) that run between build() and engine
    -- update() are reflected. Cached via rawset on first read.
    setmetatable(ctx, { __index = function(t, k)
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
    end })

    return ctx
end
