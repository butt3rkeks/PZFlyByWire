--[[
    FBWForceComputer — Pure math force computation

    Given state → force vectors. No Bullet API, no applyImpulse.
    Returns force magnitudes for the adapter to apply.

    Testable with synthetic inputs.
]]

FBWForceComputer = {}

-------------------------------------------------------------------------------------
-- Horizontal correction force computation.
--
-- When error > PD_ERROR_THRESHOLD: PD controller drives toward sim position.
-- When error <= threshold: velocity damping for the final stop.
-- FA-off: only damp in deadzone to allow free coast above it.
-------------------------------------------------------------------------------------
--- @param errX number Saturated position error X
--- @param errZ number Saturated position error Z
--- @param errRateX number Error rate X (frames)
--- @param errRateZ number Error rate Z (frames)
--- @param errMag number Error magnitude (scalar)
--- @param pgain number Proportional gain
--- @param dgain number Derivative gain
--- @param velX number Current smoothed velocity X
--- @param velZ number Current smoothed velocity Z
--- @param mass number Vehicle mass
--- @param velForceFactor number Force scaling factor (1/PHYSICS_SUBSTEP)
--- @param finalStopGain number Velocity damping gain for final stop
--- @param isFlightAssistOff boolean FA-off mode active
--- @param faDeadzone number FA-off deadzone speed (m/s)
--- @param faMinDamping number FA-off minimum damping speed (m/s)
--- @return number fx, number fz Correction forces
function FBWForceComputer.computeCorrectionForce(errX, errZ, errRateX, errRateZ, errMag,
        pgain, dgain, velX, velZ, mass, velForceFactor,
        finalStopGain, isFlightAssistOff, faDeadzone, faMinDamping)

    local pdErrorThreshold = HeliConfig.PD_ERROR_THRESHOLD

    if errMag > pdErrorThreshold then
        -- PD controller: drives toward sim position
        local fx = (errX * pgain + errRateX * dgain) * mass * velForceFactor
        local fz = (errZ * pgain + errRateZ * dgain) * mass * velForceFactor
        return fx, fz
    else
        -- Final stop: velocity damping
        local applyDamping = true
        if isFlightAssistOff then
            local speed = math.sqrt(velX * velX + velZ * velZ)
            applyDamping = (speed < faDeadzone and speed > faMinDamping)
        end

        if applyDamping then
            local damping = math.min(finalStopGain, 0.8)
            local fx = -velX * damping * mass * velForceFactor
            local fz = -velZ * damping * mass * velForceFactor
            return fx, fz
        else
            return 0, 0
        end
    end
end

-------------------------------------------------------------------------------------
-- Vertical thrust force computation.
--
-- Normal path (subSteps > 0): PD + gravity compensation.
-- High-FPS fallback (subSteps=0, physicsDelta > 0): gravity only, fractional.
-------------------------------------------------------------------------------------
--- @param desiredVelY number Target vertical velocity
--- @param savedVelY number Current vertical velocity
--- @param mass number Vehicle mass
--- @param Kp number Vertical PD gain
--- @param gravity number Gravity constant
--- @param subSteps number Integer physics sub-steps this frame
--- @param physicsDelta number Actual physics time this frame (seconds)
--- @param applyGravComp boolean Whether to add gravity compensation
--- @return number fy Vertical force
function FBWForceComputer.computeThrustForce(desiredVelY, savedVelY, mass, Kp, gravity, subSteps, physicsDelta, applyGravComp)
    if subSteps > 0 then
        local errorY = desiredVelY - savedVelY
        local thrustFy = Kp * errorY * mass * subSteps

        if applyGravComp then
            thrustFy = thrustFy + mass * gravity * subSteps
        end

        return thrustFy

    elseif applyGravComp and physicsDelta > 0 then
        -- High FPS fallback: gravity compensation only (fractional sub-step)
        local physicsSubstep = 0.01  -- PHYSICS_SUBSTEP constant
        return mass * gravity * (physicsDelta / physicsSubstep)

    else
        return 0
    end
end
