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
--- @param vehicle BaseVehicle
--- @param curr_z number Current altitude (z-levels)
--- @param freeMode boolean Flight assist off
--- @param keys HEFKeys Key state
--- @param velY number Current vertical velocity
--- @return number targetVelY, boolean gravComp, boolean vBraking, boolean engineDead
function FBWFlightModel.computeVerticalTarget(vehicle, curr_z, freeMode, keys, velY)
    local engine = vehicle:getPartById("Engine")
    local engineCondition = engine and engine:getCondition() or -1
    local fuelPercent = vehicle:getRemainingFuelPercentage()

    return VerticalModel.computeTarget(
        keys, velY, curr_z, fuelPercent, engineCondition, freeMode, getVerticalCfg())
end
