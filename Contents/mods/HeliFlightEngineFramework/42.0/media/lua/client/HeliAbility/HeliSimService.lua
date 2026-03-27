--[[
    HeliSimService — Thin dispatcher: delegates all calls to active flight engine

    Reads engine name from HeliConfig, resolves via IFlightEngine registry.
    Every public method is a 1:1 delegation to the active engine.
    No flight logic lives here.
]]

HeliSimService = {}

local _activeEngine = nil

function HeliSimService._resolveEngine()
    local name = HeliConfig.getEngineName()
    _activeEngine = IFlightEngine.get(name)
    if not _activeEngine then
        print("HEF WARNING: flight engine '" .. tostring(name) .. "' not registered. Falling back to FBW.")
        _activeEngine = IFlightEngine.get("FBW")
    end
    if not _activeEngine then
        error("HeliSimService: no flight engine available (requested '" .. tostring(name) .. "')")
    end
end

--- Lazy-resolve the active engine on first call. After resolution, _activeEngine
--- is stable for the session — engine selection only changes on save reload.
local function engine()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine
end

-- Frame updates

--- @param ctx HEFCtx
--- @return HEFUpdateResult
function HeliSimService.update(ctx)
    return engine().update(ctx)
end

--- @param ctx HEFCtx
--- @return HEFGroundResult
function HeliSimService.updateGround(ctx)
    return engine().updateGround(ctx)
end

--- @param cctx HEFCorrectionCtx
function HeliSimService.applyCorrectionForces(cctx)
    local e = engine()
    if e.applyCorrectionForces then
        return e.applyCorrectionForces(cctx)
    end
end

-- Lifecycle

function HeliSimService.resetFlightState()
    return engine().resetFlightState()
end

--- @param vehicle BaseVehicle
function HeliSimService.initFlight(vehicle)
    return engine().initFlight(vehicle)
end

function HeliSimService.tickWarmup()
    return engine().tickWarmup()
end

--- @return boolean
function HeliSimService.isWarmedUp()
    return engine().isWarmedUp()
end

-- Tunables

--- @return HEFTunable[]
function HeliSimService.getTunables()
    return engine().getTunables()
end

--- @param name string
--- @return number
function HeliSimService.getTunable(name)
    return engine().getTunable(name)
end

--- @param name string
--- @param value number
function HeliSimService.setTunable(name, value)
    return engine().setTunable(name, value)
end

-- Sandbox options

--- @return HEFSandboxOptions
function HeliSimService.getSandboxOptions()
    return engine().getSandboxOptions()
end

-- Debug

--- @return table
function HeliSimService.getDebugState()
    return engine().getDebugState()
end

--- @return string[]
function HeliSimService.getDebugColumns()
    return engine().getDebugColumns()
end

--- @return number
function HeliSimService.getIntendedYaw()
    return engine().getIntendedYaw()
end

-- Commands

--- @return HEFCommand[]
function HeliSimService.getCommands()
    return engine().getCommands()
end

--- @param name string
--- @param args string
--- @return string
function HeliSimService.executeCommand(name, args)
    return engine().executeCommand(name, args)
end

-- Metadata

--- @return HEFEngineInfo
function HeliSimService.getInfo()
    return engine().getInfo()
end
