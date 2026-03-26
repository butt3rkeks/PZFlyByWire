--[[
    FBWRawSim — Pure simulation model (ideal trajectory)

    Maintains simulated position + velocity. Advances based on desired velocity
    with virtual inertia. No PD, no error, no flags, no game API.

    Testable with numbers on paper.
]]

FBWRawSim = {}

-------------------------------------------------------------------------------------
-- Simulated model state
-------------------------------------------------------------------------------------
local _simPosX, _simPosZ = 0, 0
local _simVelX, _simVelZ = 0, 0

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Initialize position, zero velocity.
--- @param posX number Initial X position
--- @param posZ number Initial Z position
function FBWRawSim.reset(posX, posZ)
    _simPosX = posX
    _simPosZ = posZ
    _simVelX = 0
    _simVelZ = 0
end

--- Advance simulation one step: blend velocity toward desired, then integrate position.
--- @param desiredVelX number Target X velocity from tilt
--- @param desiredVelZ number Target Z velocity from tilt
--- @param dt number Time step (1/fps)
--- @param inertia number Pre-computed inertia rate (brake * accel or brake * decel)
function FBWRawSim.advance(desiredVelX, desiredVelZ, dt, inertia)
    _simVelX = _simVelX + (desiredVelX - _simVelX) * inertia
    _simVelZ = _simVelZ + (desiredVelZ - _simVelZ) * inertia
    _simPosX = _simPosX + _simVelX * dt
    _simPosZ = _simPosZ + _simVelZ * dt
end

--- Re-anchor position to actual, preserve velocity.
--- Used when heading changes significantly — old sim position was in the wrong direction.
--- @param posX number Actual X position
--- @param posZ number Actual Z position
function FBWRawSim.snapPosition(posX, posZ)
    _simPosX = posX
    _simPosZ = posZ
end

--- Gradual soft anchor: blend sim position toward actual, decay velocity.
--- Prevents stale position error during hover without hard snap discontinuity.
--- @param actualX number Actual X position
--- @param actualZ number Actual Z position
--- @param blend number Blend rate (0.1 = ~90% convergence in 22 frames)
function FBWRawSim.blendToward(actualX, actualZ, blend)
    _simPosX = _simPosX + (actualX - _simPosX) * blend
    _simPosZ = _simPosZ + (actualZ - _simPosZ) * blend
    _simVelX = _simVelX * (1 - blend)
    _simVelZ = _simVelZ * (1 - blend)
end

--- Get current simulation state.
--- @return number simPosX, number simPosZ, number simVelX, number simVelZ
function FBWRawSim.getState()
    return _simPosX, _simPosZ, _simVelX, _simVelZ
end
