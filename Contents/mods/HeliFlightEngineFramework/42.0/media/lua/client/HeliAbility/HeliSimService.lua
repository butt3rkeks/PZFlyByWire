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
        error("HeliSimService: unknown flight engine '" .. tostring(name) .. "'")
    end
end

-- Frame updates
function HeliSimService.update(ctx)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.update(ctx)
end

function HeliSimService.updateGround(ctx)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.updateGround(ctx)
end

function HeliSimService.applyCorrectionForces(vehicle)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.applyCorrectionForces(vehicle)
end

-- Lifecycle
function HeliSimService.resetFlightState()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.resetFlightState()
end

function HeliSimService.initFlight(vehicle)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.initFlight(vehicle)
end

function HeliSimService.tickWarmup()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.tickWarmup()
end

function HeliSimService.isWarmedUp()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.isWarmedUp()
end

-- Tunables
function HeliSimService.getTunables()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getTunables()
end

function HeliSimService.getTunable(name)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getTunable(name)
end

function HeliSimService.setTunable(name, value)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.setTunable(name, value)
end

-- Sandbox options
function HeliSimService.getSandboxOptions()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getSandboxOptions()
end

-- Debug
function HeliSimService.getDebugState()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getDebugState()
end

function HeliSimService.getDebugColumns()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getDebugColumns()
end

function HeliSimService.getIntendedYaw()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getIntendedYaw()
end

-- Commands
function HeliSimService.getCommands()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getCommands()
end

function HeliSimService.executeCommand(name, args)
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.executeCommand(name, args)
end

-- Metadata
function HeliSimService.getInfo()
    if not _activeEngine then HeliSimService._resolveEngine() end
    return _activeEngine.getInfo()
end
