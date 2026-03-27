--[[
    SimModel2D — 2D position/velocity simulation with inertia

    Maintains an ideal trajectory: simulated position + velocity that smoothly
    tracks a desired velocity. Used to generate position targets that a
    correction system (PD, PID, etc.) can chase.

    Instance-based: create with SimModel2D.new() so multiple can coexist.
    No game API dependencies — pure math, testable with numbers on paper.
]]

SimModel2D = {}
SimModel2D.__index = SimModel2D

--- Create a new 2D simulation model.
--- @param targetFps number The reference FPS for framerate-independent smoothing (e.g. 60)
--- @return SimModel2D
function SimModel2D.new(targetFps)
    return setmetatable({
        posX = 0, posZ = 0,
        velX = 0, velZ = 0,
        targetFps = targetFps or 60,
    }, SimModel2D)
end

--- Initialize position, zero velocity.
--- @param posX number Initial X position
--- @param posZ number Initial Z position
function SimModel2D:reset(posX, posZ)
    self.posX = posX or 0
    self.posZ = posZ or 0
    self.velX = 0
    self.velZ = 0
end

--- Advance simulation one step: blend velocity toward desired, then integrate position.
--- Uses framerate-independent exponential smoothing: at targetFps, alpha == inertia.
--- At higher FPS, each frame applies proportionally less change so convergence rate
--- is consistent regardless of frame rate.
--- @param desiredVelX number Target X velocity
--- @param desiredVelZ number Target Z velocity
--- @param dt number Time step (1/fps)
--- @param inertia number Smoothing rate (0..1, higher = faster convergence)
function SimModel2D:advance(desiredVelX, desiredVelZ, dt, inertia)
    local blendFactor = 1 - (1 - inertia) ^ (dt * self.targetFps)
    self.velX = self.velX + (desiredVelX - self.velX) * blendFactor
    self.velZ = self.velZ + (desiredVelZ - self.velZ) * blendFactor
    self.posX = self.posX + self.velX * dt
    self.posZ = self.posZ + self.velZ * dt
end

--- Re-anchor position to actual, preserve velocity.
--- Used when reference frame changes (e.g. heading re-anchor).
--- @param posX number Actual X position
--- @param posZ number Actual Z position
function SimModel2D:snapPosition(posX, posZ)
    self.posX = posX
    self.posZ = posZ
end

--- Gradual soft anchor: blend sim position toward actual, decay velocity.
--- Prevents stale position error during hover without hard snap discontinuity.
--- @param actualX number Actual X position
--- @param actualZ number Actual Z position
--- @param blend number Blend rate (0.1 = ~90% convergence in 22 frames)
function SimModel2D:blendToward(actualX, actualZ, blend)
    self.posX = self.posX + (actualX - self.posX) * blend
    self.posZ = self.posZ + (actualZ - self.posZ) * blend
    self.velX = self.velX * (1 - blend)
    self.velZ = self.velZ * (1 - blend)
end

--- Get current simulation state.
--- @return number posX, number posZ, number velX, number velZ
function SimModel2D:getState()
    return self.posX, self.posZ, self.velX, self.velZ
end
