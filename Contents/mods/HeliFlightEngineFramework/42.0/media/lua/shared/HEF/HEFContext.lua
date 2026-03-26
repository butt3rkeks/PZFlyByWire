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
--- @field heliType string Helicopter type name (key into HeliList)
--- @field curr_z number Current altitude (z-levels)
--- @field nowMaxZ number Ground height under helicopter (z-levels)
--- @field tempVector2 Vector3f Reusable scratch vector (PZ API)
--- @field velX number Smoothed horizontal velocity X (m/s)
--- @field velY number Raw vertical velocity Y (m/s)
--- @field velZ number Smoothed horizontal velocity Z (m/s)
--- @field blocked HEFBlocked Wall collision state booleans

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
    { name = "heliType",      type = _types.string,      desc = "Helicopter type name (key into HeliList)" },
    { name = "curr_z",        type = _types.number,      desc = "Current altitude (z-levels)" },
    { name = "nowMaxZ",       type = _types.number,      desc = "Ground height under helicopter (z-levels)" },
    { name = "tempVector2",   type = _types.Vector3f,    desc = "Reusable scratch vector (PZ API)" },
    { name = "velX",          type = _types.number,      desc = "Smoothed horizontal velocity X (m/s)" },
    { name = "velY",          type = _types.number,      desc = "Raw vertical velocity Y (m/s)" },
    { name = "velZ",          type = _types.number,      desc = "Smoothed horizontal velocity Z (m/s)" },
    { name = "blocked",       type = _types.bool_table,  desc = "Wall collision state booleans" },
}

-------------------------------------------------------------------------------------
-- Builder: constructs a complete HEFCtx from game state
--
-- Called once per frame by HeliMove. Reads keyboard, velocity, terrain.
-- All per-frame side effects (cache invalidation, velocity adapter state)
-- happen here so engines never need to call these APIs directly.
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

    -- Wall blocking (uses per-frame cache)
    local blocked = {
        up    = HeliTerrainUtil.isBlocked(playerObj, "UP", vehicle, tempVector2),
        down  = HeliTerrainUtil.isBlocked(playerObj, "DOWN", vehicle, tempVector2),
        left  = HeliTerrainUtil.isBlocked(playerObj, "LEFT", vehicle, tempVector2),
        right = HeliTerrainUtil.isBlocked(playerObj, "RIGHT", vehicle, tempVector2),
    }

    return {
        vehicle = vehicle,
        playerObj = playerObj,
        keys = keys,
        fpsMultiplier = fpsMultiplier,
        heliType = heliType,
        curr_z = curr_z,
        nowMaxZ = nowMaxZ,
        tempVector2 = tempVector2,
        velX = toLuaNum(rawVelX),
        velY = toLuaNum(rawVelY),
        velZ = toLuaNum(rawVelZ),
        blocked = blocked,
    }
end
