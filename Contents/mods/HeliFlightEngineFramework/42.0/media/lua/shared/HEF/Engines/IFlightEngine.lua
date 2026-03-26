--[[
    IFlightEngine — Interface definition + engine registry

    Every flight engine must implement the required methods listed below.
    At register time, all methods are validated — missing method = immediate error.
    This prevents silent failures at runtime.

    Registry:
      IFlightEngine.register("FBW", engineTable) — validates + stores
      IFlightEngine.get("FBW") — retrieves by name

    ═══════════════════════════════════════════════════════════════════════
    CONTRACT (see plan for full documentation of each method)
    ═══════════════════════════════════════════════════════════════════════

    FRAME UPDATE:
      update(ctx) → results {engineDead, dualPathActive, displaySpeed, ...}
      updateGround(ctx) → results {liftoff, displaySpeed}
      applyCorrectionForces(vehicle) — 0-frame delay force path

    LIFECYCLE:
      resetFlightState()
      initFlight(vehicle)
      tickWarmup()
      isWarmedUp() → boolean

    TUNABLES (runtime, session-only):
      getTunables() → array of {name, label, value, min, max, default}
      getTunable(name) → number
      setTunable(name, value)

    SANDBOX OPTIONS (persistent, per-save):
      getSandboxOptions() → {namespace=string, options=array of {field, type, default, ...}}

    DEBUG:
      getDebugState() → table
      getDebugColumns() → ordered array of column name strings
      getIntendedYaw() → number (degrees)

    COMMANDS:
      getCommands() → array of {name, description, args}
      executeCommand(name, argsString) → response string

    METADATA:
      getInfo() → {name, version, description}
]]

IFlightEngine = {}

IFlightEngine._engines = {}

IFlightEngine.REQUIRED_METHODS = {
    "update", "updateGround", "applyCorrectionForces",
    "resetFlightState", "initFlight", "tickWarmup", "isWarmedUp",
    "getTunables", "getTunable", "setTunable",
    "getSandboxOptions",
    "getCommands", "executeCommand",
    "getDebugState", "getDebugColumns", "getIntendedYaw", "getInfo",
}

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
--- @return table Array of engine name strings
function IFlightEngine.getRegisteredNames()
    local names = {}
    for name, _ in pairs(IFlightEngine._engines) do
        names[#names + 1] = name
    end
    return names
end
