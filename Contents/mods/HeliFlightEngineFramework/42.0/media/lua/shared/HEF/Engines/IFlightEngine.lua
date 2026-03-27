--[[
    IFlightEngine — Interface definition + engine registry

    Every flight engine must implement the required methods listed below.
    Optional methods are available for engines that need them — the framework
    checks for their existence before calling.

    At register time, required methods are validated — missing = immediate error.
    Optional methods are listed for discoverability but not enforced.

    Registry:
      IFlightEngine.register("FBW", engineTable) — validates + stores
      IFlightEngine.get("FBW") — retrieves by name

    Context contracts (inputs):
      HEFContext.lua        — @class HEFCtx + CTX_FIELDS (OnTick phase)
      HEFCorrectionCtx.lua  — @class HEFCorrectionCtx + CTX_FIELDS (OnTickEvenPaused phase)
    Result contracts: see HEFUpdateResult.lua, HEFGroundResult.lua, etc.

    Output contract (ctx closures — preferred over direct adapter/game API calls):
      ctx.applyForce(fx, fy, fz)    — apply physics force (Bullet space, adapter-wrapped)
      ctx.setAngles(x, y, z)        — set vehicle Euler angles (degrees)
      ctx.setPhysicsActive(active)  — wake/sleep Bullet physics body
      cctx.applyForce(fx, fy, fz)   — same as ctx.applyForce, for correction path

    Engines receive all needed state via ctx fields (position, velocity, mass,
    keys, terrain, fuel, engine condition, angles, physics timing) and should
    write outputs via ctx closures. The vehicle object is still available in ctx
    as an escape hatch for engine-specific state (e.g., modData), but engines
    should not call adapters or physics APIs directly.
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
-- Optional engine methods
--
-- NOT validated at register time. Framework checks existence before calling.
-- Listed here for discoverability — engine authors should know these exist.
-------------------------------------------------------------------------------------

IFlightEngine.OPTIONAL_METHODS = {
    -- 0-frame delay correction path (see HEFCorrectionCtx.lua)
    -- Called on OnTickEvenPaused (before physics step) when update() returned
    -- dualPathActive=true. Allows horizontal correction forces to be applied
    -- with zero frame delay instead of the 1-frame delay of the OnTick path.
    -- Engines that don't need dual-path timing can omit this — just return
    -- dualPathActive=false from update() and the framework never calls it.
    "applyCorrectionForces", -- (cctx:HEFCorrectionCtx)
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
