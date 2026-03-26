--[[
    IFlightEngine — Interface definition + engine registry

    Every flight engine must implement the required methods listed below.
    At register time, all methods are validated — missing method = immediate error.

    Registry:
      IFlightEngine.register("FBW", engineTable) — validates + stores
      IFlightEngine.get("FBW") — retrieves by name

    Engine authors: read CTX_FIELDS and the EmmyLua @class annotations
    for the full framework contract (what you receive, what you must return).
]]

IFlightEngine = {}

IFlightEngine._engines = {}

-------------------------------------------------------------------------------------
-- Framework-provided context: type validators for runtime checks
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

-------------------------------------------------------------------------------------
-- Framework-provided context (ctx)
--
-- Built by HeliMove once per frame, passed to update(ctx) and updateGround(ctx).
-- Engines can rely on all fields being present and correctly typed every frame.
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
--- @field up boolean
--- @field down boolean
--- @field left boolean
--- @field right boolean

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

IFlightEngine.CTX_FIELDS = {
    { name = "vehicle",       type = _types.BaseVehicle, desc = "The helicopter being flown" },
    { name = "playerObj",     type = _types.IsoPlayer,   desc = "The pilot" },
    { name = "keys",          type = _types.key_table,   desc = "{up,down,left,right,w,s,a,d} booleans — keyboard state" },
    { name = "fpsMultiplier", type = _types.number,      desc = "TARGET_FPS / actualFPS (frame scaling)" },
    { name = "heliType",      type = _types.string,      desc = "Helicopter type name (key into HeliList)" },
    { name = "curr_z",        type = _types.number,      desc = "Current altitude (z-levels)" },
    { name = "nowMaxZ",       type = _types.number,      desc = "Ground height under helicopter (z-levels)" },
    { name = "tempVector2",   type = _types.Vector3f,    desc = "Reusable scratch vector (PZ API)" },
    { name = "velX",          type = _types.number,      desc = "Smoothed horizontal velocity X (m/s)" },
    { name = "velY",          type = _types.number,      desc = "Raw vertical velocity Y (m/s)" },
    { name = "velZ",          type = _types.number,      desc = "Smoothed horizontal velocity Z (m/s)" },
    { name = "blocked",       type = _types.bool_table,  desc = "{up,down,left,right} booleans — wall collision state" },
}

-------------------------------------------------------------------------------------
-- Required engine methods
--
-- Validated at register time — missing method = immediate error.
-------------------------------------------------------------------------------------

--- @class HEFTunable
--- @field name string Short identifier (e.g. "pgain")
--- @field label string Human-readable label (e.g. "P Gain")
--- @field value number Current value
--- @field min number Minimum allowed value
--- @field max number Maximum allowed value
--- @field default number Default value

--- @class HEFSandboxOption
--- @field field string Sandbox field name (e.g. "GravityEstimate")
--- @field type string "double"|"integer"|"boolean"|"enum"
--- @field default number|boolean Default value
--- @field min number|nil Min (double/integer only)
--- @field max number|nil Max (double/integer only)
--- @field numValues number|nil Enum count (enum only)
--- @field desc string Human-readable description

--- @class HEFSandboxOptions
--- @field namespace string Sandbox namespace owned by this engine (e.g. "FBW")
--- @field options HEFSandboxOption[]

--- @class HEFCommand
--- @field name string Command identifier (e.g. "recalibrate")
--- @field description string What the command does
--- @field args string Expected arguments format (e.g. "" or "<degrees>")

--- @class HEFEngineInfo
--- @field name string Engine identifier (e.g. "FBW")
--- @field version string Version string (e.g. "1.0")
--- @field description string Human-readable description

--- @class HEFUpdateResult
--- @field engineDead boolean Engine below death threshold — framework shuts off engine
--- @field dualPathActive boolean True = framework fires applyCorrectionForces this frame
--- @field displaySpeed number km/h for speedometer (vehicle:setSpeedKmHour)

--- @class HEFGroundResult
--- @field liftoff boolean True = framework transitions to warmup/airborne
--- @field displaySpeed number km/h for speedometer

IFlightEngine.REQUIRED_METHODS = {
    -- Frame update
    "update",                -- (ctx:HEFCtx) → HEFUpdateResult
    "updateGround",          -- (ctx:HEFCtx) → HEFGroundResult
    "applyCorrectionForces", -- (vehicle:BaseVehicle) — 0-frame delay force path
    -- Lifecycle
    "resetFlightState",      -- ()
    "initFlight",            -- (vehicle:BaseVehicle)
    "tickWarmup",            -- ()
    "isWarmedUp",            -- () → boolean
    -- Tunables (runtime, session-only)
    "getTunables",           -- () → HEFTunable[]
    "getTunable",            -- (name:string) → number
    "setTunable",            -- (name:string, value:number)
    -- Sandbox options (persistent, per-save)
    "getSandboxOptions",     -- () → HEFSandboxOptions
    -- Debug
    "getDebugState",         -- () → table
    "getDebugColumns",       -- () → string[]
    "getIntendedYaw",        -- () → number (degrees)
    -- Commands
    "getCommands",           -- () → HEFCommand[]
    "executeCommand",        -- (name:string, argsString:string) → string
    -- Metadata
    "getInfo",               -- () → HEFEngineInfo
}

-------------------------------------------------------------------------------------
-- Registry
-------------------------------------------------------------------------------------

--- Register a flight engine. Validates all required methods at load time.
--- @param name string Engine identifier (e.g. "FBW")
--- @param engineTable table Table implementing all required methods
function IFlightEngine.register(name, engineTable)
    for _, method in ipairs(IFlightEngine.REQUIRED_METHODS) do
        if type(engineTable[method]) ~= "function" then
            error("IFlightEngine: engine '" .. name .. "' missing required method: " .. method)
        end
    end
    IFlightEngine._engines[name] = engineTable
end

--- Get a registered engine by name.
--- @param name string Engine identifier
--- @return table|nil Engine table or nil if not registered
function IFlightEngine.get(name)
    return IFlightEngine._engines[name]
end

--- Get all registered engine names.
--- @return string[]
function IFlightEngine.getRegisteredNames()
    local names = {}
    for name, _ in pairs(IFlightEngine._engines) do
        names[#names + 1] = name
    end
    return names
end
