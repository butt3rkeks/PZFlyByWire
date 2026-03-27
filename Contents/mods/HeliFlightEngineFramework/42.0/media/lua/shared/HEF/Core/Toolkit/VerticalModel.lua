--[[
    VerticalModel — Default helicopter vertical flight behavior

    Computes vertical velocity targets from pilot input (W/S keys),
    engine condition, fuel state, and altitude.

    This is the standard helicopter vertical model:
      W = ascend, S = descend, nothing = hover, no fuel = fall, engine dead = fast fall.

    Stateless utility. Engines can use this as-is or replace with custom logic.
    All thresholds and speeds are passed as a config table — no HeliConfig dependency.
]]

VerticalModel = {}

--- @class VerticalConfig
--- @field ascendSpeed number Ascend velocity (Bullet Y/s)
--- @field descendSpeed number Descend velocity (Bullet Y/s)
--- @field gravityFallSpeed number Engine-off fall velocity (Bullet Y/s)
--- @field engineDeadFallSpeed number Engine-dead fall velocity (Bullet Y/s)
--- @field engineDeadCondition number Engine condition threshold (0..1)
--- @field minDescentAltitude number Minimum altitude for descent/fall (z-levels)
--- @field maxAltitude number Flight ceiling (z-levels)
--- @field ceilingZoneHeight number Height of ceiling taper zone (z-levels)

--- @class VerticalResult
--- @field targetVelY number Target vertical velocity
--- @field gravComp boolean Apply gravity compensation
--- @field vBraking boolean Apply vertical velocity braking (hover damping)
--- @field engineDead boolean Engine is destroyed

--- Compute vertical velocity target from flight state.
--- @param keys HEFKeys Pilot key state
--- @param velY number Current vertical velocity
--- @param curr_z number Current altitude (z-levels)
--- @param fuelPercent number Remaining fuel (0..100)
--- @param engineCondition number Engine part condition (0..100, or -1 if no engine part)
--- @param freeMode boolean Flight assist off (coast instead of hover)
--- @param cfg VerticalConfig Tuning parameters
--- @return number targetVelY, boolean gravComp, boolean vBraking, boolean engineDead
function VerticalModel.computeTarget(keys, velY, curr_z, fuelPercent, engineCondition, freeMode, cfg)
    -- 1. Engine dead — fast uncontrolled fall
    if engineCondition >= 0 and engineCondition < cfg.engineDeadCondition then
        return -cfg.engineDeadFallSpeed, false, false, true
    end

    -- 2. S key — controlled descent
    if keys.s and curr_z > cfg.minDescentAltitude then
        return -cfg.descendSpeed, true, false, false
    end

    -- 3. No fuel — gravity fall
    if fuelPercent <= 0 and curr_z > cfg.minDescentAltitude then
        return -cfg.gravityFallSpeed, false, false, false
    end

    -- 4. W key — ascend (with ceiling taper)
    if keys.w and curr_z < cfg.maxAltitude and fuelPercent > 0 then
        local speed = cfg.ascendSpeed
        local ceilingStart = cfg.maxAltitude - cfg.ceilingZoneHeight
        if curr_z > ceilingStart then
            local ceilingFactor = (cfg.maxAltitude - curr_z) / cfg.ceilingZoneHeight
            speed = speed * math.max(ceilingFactor, 0)
        end
        return speed, true, false, false
    end

    -- 5. No vertical input
    if freeMode then
        -- Coast at current velocity
        return velY, true, false, false
    end

    -- Hover
    return 0, true, true, false
end
