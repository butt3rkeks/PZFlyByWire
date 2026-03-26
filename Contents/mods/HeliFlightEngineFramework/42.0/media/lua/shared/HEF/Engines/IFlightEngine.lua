--[[
    IFlightEngine — Interface definition + engine registry

    Every flight engine must implement the required methods listed below.
    At register time, all methods are validated — missing method = immediate error.

    Registry:
      IFlightEngine.register("FBW", engineTable) — validates + stores
      IFlightEngine.get("FBW") — retrieves by name

    Context contract: see HEFContext.lua for @class HEFCtx and CTX_FIELDS.
    Result contracts: see @class annotations below, adjacent to each method group.
]]

IFlightEngine = {}

IFlightEngine._engines = {}

-------------------------------------------------------------------------------------
-- Required engine methods
--
-- Validated at register time — missing method = immediate error.
-------------------------------------------------------------------------------------

IFlightEngine.REQUIRED_METHODS = {
    -- Frame update (see HEFUpdateResult.lua, HEFGroundResult.lua)
    "update",                -- (ctx:HEFCtx) → HEFUpdateResult
    "updateGround",          -- (ctx:HEFCtx) → HEFGroundResult
    "applyCorrectionForces", -- (vehicle:BaseVehicle) — 0-frame delay force path
    -- Lifecycle
    "resetFlightState",      -- ()
    "initFlight",            -- (vehicle:BaseVehicle)
    "tickWarmup",            -- ()
    "isWarmedUp",            -- () → boolean
    -- Tunables (see HEFTunable.lua)
    "getTunables",           -- () → HEFTunable[]
    "getTunable",            -- (name:string) → number
    "setTunable",            -- (name:string, value:number)
    -- Sandbox options (see HEFSandboxOptions.lua)
    "getSandboxOptions",     -- () → HEFSandboxOptions
    -- Debug
    "getDebugState",         -- () → table
    "getDebugColumns",       -- () → string[]
    "getIntendedYaw",        -- () → number (degrees)
    -- Commands (see HEFCommand.lua)
    "getCommands",           -- () → HEFCommand[]
    "executeCommand",        -- (name:string, argsString:string) → string
    -- Metadata (see HEFEngineInfo.lua)
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
