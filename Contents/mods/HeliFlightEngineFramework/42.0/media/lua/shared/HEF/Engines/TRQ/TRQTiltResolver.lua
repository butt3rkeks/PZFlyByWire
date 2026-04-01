--[[
    TRQTiltResolver — Resolve body tilt into desired horizontal velocity

    Initially identical to FBWTiltResolver. Uses FlightFilters (Core/Toolkit)
    for the filter pipeline. Can be refined later for torque-specific behavior.

    Orchestrates: wall pre-blocking → FlightFilters pipeline (noise floor →
    thrust decomposition → speed clamp → direction clamp).
]]

TRQTiltResolver = {}

-------------------------------------------------------------------------------------
-- State: previous position for direction clamp (one frame lookback)
-------------------------------------------------------------------------------------
local _prevPosX, _prevPosZ = nil, nil

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset state for new flight session.
function TRQTiltResolver.reset()
    _prevPosX, _prevPosZ = nil, nil
end

--- Resolve body angles + wall blocking into a desired horizontal velocity vector.
--- @param angleZ number Body pitch angle (radians)
--- @param angleX number Body roll angle (radians)
--- @param blocked table Framework-provided blocked directions
--- @param fwdX number Forward direction X
--- @param fwdZ number Forward direction Z
--- @param posX number Current world X position
--- @param posZ number Current world Z position
--- @return number velX, number velZ, number speed, number totalTiltRad, boolean isBlockedHit
function TRQTiltResolver.resolve(angleZ, angleX, blocked, fwdX, fwdZ, posX, posZ)
    local angle_90 = math.rad(90)
    local pitchDev = angleZ - angle_90
    local rollDev  = angleX - angle_90
    local isBlockedHit = false

    local cosZ = math.cos(angleZ)
    local cosX = math.cos(angleX)

    if math.abs(cosZ) > HeliConfig.DIRECTION_COS_THRESHOLD then
        local pitchBlocked = (cosZ < 0) and blocked.up or blocked.down
        if pitchBlocked then pitchDev = 0; isBlockedHit = true end
    end
    if math.abs(cosX) > HeliConfig.DIRECTION_COS_THRESHOLD then
        local rollBlocked = (cosX < 0) and blocked.right or blocked.left
        if rollBlocked then rollDev = 0; isBlockedHit = true end
    end

    local totalTiltRad = math.sqrt(pitchDev * pitchDev + rollDev * rollDev)

    local noiseFloor = HeliConfig.TILT_NOISE_FLOOR
    local maxHSpeed  = HeliConfig.GetMaxHorizontalSpeed()

    local effectiveTilt = FlightFilters.applyNoiseFloor(totalTiltRad, noiseFloor)

    local velX, velZ, speed = FlightFilters.decomposeThrustDirection(
        pitchDev, rollDev, totalTiltRad, fwdX, fwdZ, maxHSpeed, effectiveTilt)

    velX, velZ, speed = FlightFilters.clampSpeed(velX, velZ, maxHSpeed)

    if _prevPosX and speed > 0.1 then
        local dx = posX - _prevPosX
        local dz = posZ - _prevPosZ
        local moveMag = math.sqrt(dx * dx + dz * dz)
        if moveMag > 0.08 then
            local moveAngle = math.atan2(dz, dx)
            velX, velZ = FlightFilters.clampThrustDirection(
                velX, velZ, speed, moveAngle, moveMag, HeliConfig.MAX_THRUST_LEAD)
        end
    end
    _prevPosX, _prevPosZ = posX, posZ

    return velX, velZ, speed, totalTiltRad, isBlockedHit
end
