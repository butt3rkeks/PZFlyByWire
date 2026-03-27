--[[
    GroundModel — Default helicopter ground behavior

    Handles the "on the ground" state: hold position (kill residual velocity),
    apply liftoff force when W is pressed with fuel.

    Stateless utility. Engines can use this as-is or replace with custom logic
    (e.g., ground taxi physics).
]]

GroundModel = {}

--- @class GroundConfig
--- @field velocityKillFactor number Force multiplier for zeroing ground velocity (e.g. 100)
--- @field velocityThreshold number Minimum velocity magnitude to apply kill force (e.g. 0.01)
--- @field ascendSpeed number Initial liftoff speed (Bullet Y/s)
--- @field gravity number Gravity for compensation during liftoff
--- @field verticalGain number PD gain for liftoff thrust

--- Compute ground forces. Returns liftoff flag and optional force to apply.
--- @param ctx HEFCtx Framework context
--- @param cfg GroundConfig Tuning parameters
--- @return HEFGroundResult
function GroundModel.update(ctx, cfg)
    local keys = ctx.keys
    local mass = ctx.mass
    local velX, velY, velZ = ctx.velX, ctx.velY, ctx.velZ
    local groundVelocityMagnitude = math.abs(velX) + math.abs(velY) + math.abs(velZ)

    local liftoff = false

    if keys.w and ctx.fuelPercent > 0 then
        ctx.setPhysicsActive(true)
        if ctx.subSteps > 0 then
            local liftoffForceY = cfg.verticalGain * (cfg.ascendSpeed - velY) * mass * ctx.subSteps
                               + mass * cfg.gravity * ctx.subSteps
            ctx.applyForce(
                -velX * mass * cfg.velocityKillFactor,
                liftoffForceY,
                -velZ * mass * cfg.velocityKillFactor)
        end
        liftoff = true
    elseif groundVelocityMagnitude > cfg.velocityThreshold then
        ctx.applyForce(
            -velX * mass * cfg.velocityKillFactor,
            -velY * mass * cfg.velocityKillFactor,
            -velZ * mass * cfg.velocityKillFactor)
    end

    return {
        liftoff = liftoff,
        displaySpeed = 0,
    }
end
