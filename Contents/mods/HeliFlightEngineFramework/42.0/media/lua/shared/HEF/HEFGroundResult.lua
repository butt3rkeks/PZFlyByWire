--[[
    HEFGroundResult — Return type for IHEFEngine:updateGround(ctx)
]]

--- @class HEFGroundResult
--- @field liftoff boolean True = framework transitions to warmup/airborne
--- @field displaySpeed number km/h for speedometer

HEFGroundResult = {}

--- @param liftoff boolean
--- @param displaySpeed number
--- @return HEFGroundResult
function HEFGroundResult.new(liftoff, displaySpeed)
    return {
        liftoff = liftoff,
        displaySpeed = displaySpeed,
    }
end
