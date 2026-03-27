--[[
    HeliForceAdapter — Bullet physics force output + sub-step accounting

    I/O boundary with Bullet: handles coordinate swaps (PZ→Bullet Y/Z),
    applyImpulseGeneric multiplier, and physics sub-step accumulator.

    All Bullet-specific knowledge lives here. Core modules never touch Bullet.
]]

HeliForceAdapter = {}

-------------------------------------------------------------------------------------
-- Bullet constants
-------------------------------------------------------------------------------------
local PHYSICS_SUBSTEP = 0.01
local MIN_FORCE_MAG = 0.001
local IMPULSE_GENERIC_MULTIPLIER = 30.0

-------------------------------------------------------------------------------------
-- Physics sub-step accumulator
-------------------------------------------------------------------------------------
local _physicsLocalTime = 0
local _lastSubSteps = 0

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Apply force to vehicle via applyImpulseGeneric (Bullet coordinate space).
--- Handles Y/Z swap and magnitude→direction+strength decomposition.
--- @param vehicle BaseVehicle
--- @param bulletFx number Force X (Bullet space)
--- @param bulletFy number Force Y (Bullet space, height)
--- @param bulletFz number Force Z (Bullet space)
function HeliForceAdapter.applyForceImmediate(vehicle, bulletFx, bulletFy, bulletFz)
    local forceMagnitude = HeliUtil.toLuaNum(math.sqrt(bulletFx * bulletFx + bulletFy * bulletFy + bulletFz * bulletFz))
    if forceMagnitude < MIN_FORCE_MAG then return end

    local strength = forceMagnitude / IMPULSE_GENERIC_MULTIPLIER
    local dirX = bulletFx / forceMagnitude
    local dirY = bulletFy / forceMagnitude
    local dirZ = bulletFz / forceMagnitude

    -- applyImpulseGeneric swaps Y/Z internally: set(dirX, dirZ, dirY)
    vehicle:applyImpulseGeneric(
        vehicle:getX(), vehicle:getY(), vehicle:getZ(),
        dirX, dirZ, dirY,
        strength
    )
end

--- Compute physics sub-steps for this frame. Advances the accumulator.
--- @return number subSteps Integer sub-steps this frame
--- @return number physicsDelta Actual physics time (seconds)
function HeliForceAdapter.getSubStepsThisFrame()
    local gameTime = getGameTime and getGameTime()
    if gameTime and gameTime.getPhysicsSecondsSinceLastUpdate then
        local physicsDelta = HeliUtil.toLuaNum(gameTime:getPhysicsSecondsSinceLastUpdate())
        _physicsLocalTime = _physicsLocalTime + physicsDelta
        if _physicsLocalTime >= PHYSICS_SUBSTEP then
            local numSteps = math.floor(_physicsLocalTime / PHYSICS_SUBSTEP)
            _physicsLocalTime = _physicsLocalTime - numSteps * PHYSICS_SUBSTEP
            _lastSubSteps = numSteps
            return numSteps, physicsDelta
        end
        _lastSubSteps = 0
        return 0, physicsDelta
    end
    local fps = math.max(getAverageFPS(), HeliConfig.MIN_FPS)
    local estimatedDelta = 1.0 / fps
    _lastSubSteps = math.ceil(estimatedDelta / PHYSICS_SUBSTEP)
    return _lastSubSteps, estimatedDelta
end

--- Get sub-step count from the last getSubStepsThisFrame() call.
--- Safe to call multiple times per frame without advancing the accumulator.
--- @return number Integer sub-steps
function HeliForceAdapter.getLastSubSteps()
    return _lastSubSteps
end

--- Reset physics time accumulator.
function HeliForceAdapter.resetPhysicsTime()
    _physicsLocalTime = 0
    _lastSubSteps = 0
end
