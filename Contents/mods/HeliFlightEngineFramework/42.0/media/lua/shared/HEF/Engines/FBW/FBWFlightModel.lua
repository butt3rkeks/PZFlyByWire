--[[
    FBWFlightModel — Vertical flight behavior (thin wrapper around Toolkit/VerticalModel)

    Reads tuning parameters from HeliConfig getters, reads vehicle state
    from ctx fields, delegates to VerticalModel.computeTarget().
]]

FBWFlightModel = {}

-- Reusable config table — repopulated each call to reflect runtime overrides.
-- Safe because VerticalModel.computeTarget consumes it synchronously.
local _verticalCfg = {}

local function getVerticalCfg()
    _verticalCfg.ascendSpeed        = HeliConfig.GetAscend()
    _verticalCfg.descendSpeed       = HeliConfig.GetDescend()
    _verticalCfg.gravityFallSpeed   = HeliConfig.GetEngineOffFallSpeed()
    _verticalCfg.engineDeadFallSpeed = HeliConfig.GetEngineDeadFallSpeed()
    _verticalCfg.engineDeadCondition = HeliConfig.GetEngineDeadCondition()
    _verticalCfg.minDescentAltitude = HeliConfig.MIN_DESCENT_ALTITUDE
    _verticalCfg.maxAltitude        = HeliConfig.GetMaxAltitude()
    _verticalCfg.ceilingZoneHeight  = HeliConfig.CEILING_ZONE_HEIGHT
    return _verticalCfg
end

--- Compute vertical velocity target and gravity compensation flag.
--- @param ctx HEFCtx Framework context (provides keys, velY, currentAltitude, fuelPercent, engineCondition)
--- @param freeMode boolean Flight assist off
--- @return number targetVelY, boolean gravComp, boolean vBraking, boolean engineDead
function FBWFlightModel.computeVerticalTarget(ctx, freeMode)
    return VerticalModel.computeTarget(
        ctx.keys, ctx.velY, ctx.currentAltitude, ctx.fuelPercent, ctx.engineCondition, freeMode, getVerticalCfg())
end
