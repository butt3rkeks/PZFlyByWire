--[[
    FBWFlightModel — Vertical flight behavior (thin wrapper around Toolkit/VerticalModel)

    Reads tuning parameters from HeliConfig, extracts vehicle state,
    delegates to VerticalModel.computeTarget().
]]

FBWFlightModel = {}

-- Cached config table (rebuilt when HeliConfig values change via /hef commands)
local _verticalCfg = nil

local function getVerticalCfg()
    -- Rebuild each call — HeliConfig.get() checks runtime overrides
    return {
        ascendSpeed        = HeliConfig.get("ascend"),
        descendSpeed       = HeliConfig.get("descend"),
        gravityFallSpeed   = HeliConfig.get("fall"),
        engineDeadFallSpeed = HeliConfig.get("deadfall"),
        engineDeadCondition = HeliConfig.get("enginedead"),
        minDescentAltitude = HeliConfig.MIN_DESCENT_ALTITUDE,
        maxAltitude        = HeliConfig.get("maxalt"),
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
