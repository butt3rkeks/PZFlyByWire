--[[
    HEFUpdateResult — Return type for IHEFEngine:update(ctx)
]]

--- @class HEFUpdateResult
--- @field engineDead boolean Engine below death threshold — framework shuts off engine
--- @field dualPathActive boolean True = framework fires applyCorrectionForces this frame
--- @field displaySpeed number km/h for speedometer (vehicle:setSpeedKmHour)

HEFUpdateResult = {}

--- @param engineDead boolean
--- @param dualPathActive boolean
--- @param displaySpeed number
--- @return HEFUpdateResult
function HEFUpdateResult.new(engineDead, dualPathActive, displaySpeed)
    return {
        engineDead = engineDead,
        dualPathActive = dualPathActive,
        displaySpeed = displaySpeed,
    }
end
