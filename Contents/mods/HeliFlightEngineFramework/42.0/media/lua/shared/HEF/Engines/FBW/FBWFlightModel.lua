--[[
    FBWFlightModel — Vertical flight behavior (thin wrapper around Toolkit/VerticalModel)

    Reads tuning parameters from HeliConfig getters, reads vehicle state
    from ctx fields, delegates to VerticalModel.computeTarget().
]]

FBWFlightModel = {}

local function getVerticalCfg()
    -- Rebuild each call — getters check runtime overrides
    return {
        ascendSpeed        = HeliConfig.GetAscend(),
        descendSpeed       = HeliConfig.GetDescend(),
        gravityFallSpeed   = HeliConfig.GetFall(),
        engineDeadFallSpeed = HeliConfig.GetDeadfall(),
        engineDeadCondition = HeliConfig.GetEnginedead(),
        minDescentAltitude = HeliConfig.MIN_DESCENT_ALTITUDE,
        maxAltitude        = HeliConfig.GetMaxalt(),
        ceilingZoneHeight  = HeliConfig.CEILING_ZONE_HEIGHT,
    }
end

--- Compute vertical velocity target and gravity compensation flag.
--- @param ctx HEFCtx Framework context (provides keys, velY, curr_z, fuelPercent, engineCondition)
--- @param freeMode boolean Flight assist off
--- @return number targetVelY, boolean gravComp, boolean vBraking, boolean engineDead
function FBWFlightModel.computeVerticalTarget(ctx, freeMode)
    return VerticalModel.computeTarget(
        ctx.keys, ctx.velY, ctx.curr_z, ctx.fuelPercent, ctx.engineCondition, freeMode, getVerticalCfg())
end
