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

-- Frame updates

--- @param ctx HEFCtx
--- @return HEFUpdateResult
function HeliSimService.update(ctx)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.update(ctx)
end

--- @param ctx HEFCtx
--- @return HEFGroundResult
function HeliSimService.updateGround(ctx)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.updateGround(ctx)
end

--- @param cctx HEFCorrectionCtx
function HeliSimService.applyCorrectionForces(cctx)
    if not _activeEngine then HeliSimService._resolveEngine() end
    if _activeEngine.applyCorrectionForces then
        return _activeEngine.applyCorrectionForces(cctx)
    end
end

-- Lifecycle

function HeliSimService.resetFlightState()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.resetFlightState()
end

--- @param vehicle BaseVehicle
function HeliSimService.initFlight(vehicle)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.initFlight(vehicle)
end

function HeliSimService.tickWarmup()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.tickWarmup()
end

--- @return boolean
function HeliSimService.isWarmedUp()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.isWarmedUp()
end

-- Tunables

--- @return HEFTunable[]
function HeliSimService.getTunables()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getTunables()
end

--- @param name string
--- @return number
function HeliSimService.getTunable(name)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getTunable(name)
end

--- @param name string
--- @param value number
function HeliSimService.setTunable(name, value)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.setTunable(name, value)
end

-- Sandbox options

--- @return HEFSandboxOptions
function HeliSimService.getSandboxOptions()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getSandboxOptions()
end

-- Debug

--- @return table
function HeliSimService.getDebugState()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getDebugState()
end

--- @return string[]
function HeliSimService.getDebugColumns()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getDebugColumns()
end

--- @return number
function HeliSimService.getIntendedYaw()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getIntendedYaw()
end

-- Commands

--- @return HEFCommand[]
function HeliSimService.getCommands()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getCommands()
end

--- @param name string
--- @param args string
--- @return string
function HeliSimService.executeCommand(name, args)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.executeCommand(name, args)
end

-- Metadata

--- @return HEFEngineInfo
function HeliSimService.getInfo()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getInfo()
end
