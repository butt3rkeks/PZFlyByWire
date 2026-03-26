--[[
    FBWFlightModel — Vertical flight behavior

    Handles:
    - Vertical velocity targets from key state, fuel, engine condition

    Reads tuning parameters from HeliConfig.
    Knows about: input keys, vehicle fuel/engine state
    Does NOT know about: Bullet physics, rotation, heading, horizontal thrust
]]

FBWFlightModel = {}

-------------------------------------------------------------------------------------
-- Compute vertical velocity target and gravity compensation flag.
--
-- Priority order (highest first):
--   1. Engine dead (condition < ENGINE_DEAD_CONDITION) → fast fall, no gravity comp
--   2. S key pressed (z > MIN_DESCENT_ALTITUDE) → controlled descent, with gravity comp
--   3. No fuel (z > MIN_DESCENT_ALTITUDE) → gravity fall, no gravity comp
--   4. W key pressed (z < MAX_ALTITUDE, fuel > 0) → ascend, with gravity comp
--   5. No input:
--      - Normal mode → hover (target 0), gravity comp, braking
--      - Free mode (AutoBalance) → coast (target = current vel), gravity comp, no braking
--
-- @param vehicle BaseVehicle
-- @param curr_z number Current altitude (z-levels)
-- @param freeMode boolean Flight assist off
-- @param keys table Key state table { w, s, ... }
-- @param velY number Current vertical velocity (Lua number, already coerced)
-------------------------------------------------------------------------------------
function FBWFlightModel.computeVerticalTarget(vehicle, curr_z, freeMode, keys, velY)
    local ascendSpeed      = HeliConfig.get("ascend")
    local descendSpeed     = HeliConfig.get("descend")
    local gravityFallSpeed = HeliConfig.get("fall")
    local engineDeadFall   = HeliConfig.get("deadfall")

    -- 1. Engine dead
    local engine = vehicle:getPartById("Engine")
    if engine and engine:getCondition() < HeliConfig.ENGINE_DEAD_CONDITION then
        return -engineDeadFall, false, false, true
    end

    -- 2. S key pressed — controlled descent
    if keys.s and curr_z > HeliConfig.MIN_DESCENT_ALTITUDE then
        return -descendSpeed, true, false, false
    end

    -- 3. No fuel — gravity fall
    if vehicle:getRemainingFuelPercentage() <= 0 and curr_z > HeliConfig.MIN_DESCENT_ALTITUDE then
        return -gravityFallSpeed, false, false, false
    end

    -- 4. W key pressed — ascend (with ceiling taper to prevent oscillation at max altitude)
    if keys.w and curr_z < HeliConfig.MAX_ALTITUDE and vehicle:getRemainingFuelPercentage() > 0 then
        local ceilingStart = HeliConfig.MAX_ALTITUDE - HeliConfig.CEILING_ZONE_HEIGHT
        if curr_z > ceilingStart then
            -- Taper: full speed at ceilingStart, zero at MAX_ALTITUDE
            local ceilingFactor = (HeliConfig.MAX_ALTITUDE - curr_z) / HeliConfig.CEILING_ZONE_HEIGHT
            ascendSpeed = ascendSpeed * math.max(ceilingFactor, 0)
        end
        return ascendSpeed, true, false, false
    end

    -- 5. No vertical input
    if freeMode then
        return velY, true, false, false
    end

    -- Normal mode: hover
    return 0, true, true, false
end
