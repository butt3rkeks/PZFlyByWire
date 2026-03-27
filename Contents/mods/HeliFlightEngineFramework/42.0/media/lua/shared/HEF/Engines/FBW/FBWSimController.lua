--[[
    FBWSimController — Per-frame reference trajectory management

    Answers: "Given what the pilot wants, how does the reference sim evolve this frame?"

    Owns: FA-off coast override (step 10), inertia selection (step 11),
          sim advance (step 12), heading reanchor (step 13), soft anchor blend (step 14).

    Operates on SimModel2D and ErrorTracker2D instances passed in by FBWEngine
    (does not own them — FBWEngine also needs them for error tracking, debug,
    and correction forces).

    Stateless module: all per-frame state is passed via parameters.
    No file-scope locals except the module table.
]]

FBWSimController = {}

-------------------------------------------------------------------------------------
-- Step 10: FA-off coast logic
--
-- When FA-off (freeMode) and no tilt input: coast on sim velocity instead of
-- braking to zero. Below FA_OFF_DEADZONE actual speed: reinit sim (full stop).
--
-- @return desiredHX, desiredHZ, hasHInput (hasHInput may be mutated to true)
-------------------------------------------------------------------------------------

--- Resolve the desired velocity for the sim model, handling FA-off coast override.
---
--- @param hasHInput boolean Whether tilt input is active (from step 9)
--- @param totalVelX number Desired velocity X from tilt resolver
--- @param totalVelZ number Desired velocity Z from tilt resolver
--- @param freeMode boolean Flight assist off mode
--- @param noInput boolean No effective tilt input (from step 9)
--- @param sim SimModel2D The sim model instance (read sim velocity for coast)
--- @param actualVelX number Actual vehicle velocity X (from ctx)
--- @param actualVelZ number Actual vehicle velocity Z (from ctx)
--- @param reinitSim function Callback to reinit sim at position: reinitSim(posX, posZ)
--- @param posX number Current position X (for reinit on deadzone stop)
--- @param posZ number Current position Z (for reinit on deadzone stop)
--- @return number desiredHX, number desiredHZ, boolean hasHInput
function FBWSimController.resolveDesiredVelocity(hasHInput, totalVelX, totalVelZ,
        freeMode, noInput, sim, actualVelX, actualVelZ, reinitSim, posX, posZ)
    local desiredHX, desiredHZ = 0, 0
    if hasHInput then
        desiredHX, desiredHZ = totalVelX, totalVelZ
    end

    if freeMode and noInput then
        local _, _, svx, svz = sim:getState()
        local actualSpeed = VelocityUtil.horizontalSpeed(actualVelX, actualVelZ)
        if actualSpeed < HeliConfig.FA_OFF_DEADZONE then
            desiredHX, desiredHZ = 0, 0
            reinitSim(posX, posZ)
        else
            desiredHX, desiredHZ = svx, svz
        end
        hasHInput = true
    end

    return desiredHX, desiredHZ, hasHInput
end

-------------------------------------------------------------------------------------
-- Steps 12-14: Sim advance + heading reanchor + soft anchor
--
-- Must be called in this exact order — advance first, then conditional
-- corrections. Reanchor and soft anchor are mutually exclusive in practice
-- (reanchor fires on heading change, soft anchor fires at low speed hover).
-------------------------------------------------------------------------------------

--- Advance the reference trajectory and apply anchoring corrections.
---
--- @param sim SimModel2D The sim model instance
--- @param errorTracker ErrorTracker2D The error tracker instance (cleared on reanchor)
--- @param desiredHX number Desired horizontal velocity X (from resolveDesiredVelocity)
--- @param desiredHZ number Desired horizontal velocity Z (from resolveDesiredVelocity)
--- @param deltaTime number Frame time step (1/fps)
--- @param effectiveInertia number Pre-computed inertia (brake * accel or brake * decel)
--- @param hasHInput boolean Whether horizontal input is active (after FA-off mutation)
--- @param posX number Current actual position X
--- @param posZ number Current actual position Z
--- @param fps number Current frame rate (for reanchor threshold scaling)
--- @param flightAssistOff boolean FA-off mode active (skip reanchor to preserve coast)
--- @param positionDeltaSpeed number Position delta speed from framework (ctx.positionDeltaSpeed)
function FBWSimController.advanceAndAnchor(sim, errorTracker, desiredHX, desiredHZ,
        deltaTime, effectiveInertia, hasHInput, posX, posZ, fps, flightAssistOff, positionDeltaSpeed)
    -- Step 12: Advance sim model
    sim:advance(desiredHX, desiredHZ, deltaTime, effectiveInertia)

    -- Step 13: Heading re-anchor (skip in FA-off — coast must survive turns)
    if not flightAssistOff then
        if FBWYawController.checkHeadingReanchor(fps) then
            sim:snapPosition(posX, posZ)
            errorTracker:clearHistory()
        end
    end

    -- Step 14: Soft anchor — low speed + no input → blend sim toward actual.
    -- Prevents stale position error during hover without hard snap discontinuity.
    local desiredMag = math.sqrt(desiredHX * desiredHX + desiredHZ * desiredHZ)
    if desiredMag < HeliConfig.SIM_SNAP_THRESHOLD and not hasHInput and positionDeltaSpeed < HeliConfig.SIM_SNAP_THRESHOLD then
        sim:blendToward(posX, posZ, HeliConfig.SIM_BLEND_RATE)
    end
end
