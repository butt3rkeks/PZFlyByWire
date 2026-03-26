--[[
    HeliFilters — Composable filter pipeline for horizontal thrust

    All pure stateless functions. No state, no game API, no config reads.
    Parameters passed in by caller (HeliSimService facade).

    Pipeline: noiseFloor → thrustDecomposition → speedClamp → directionClamp
]]

HeliFilters = {}

-------------------------------------------------------------------------------------
-- Noise floor: ignore tilt below threshold, smooth ramp above it.
-- Returns effective tilt magnitude after filtering.
-------------------------------------------------------------------------------------
--- @param totalTiltRad number Combined pitch+roll tilt magnitude (radians)
--- @param noiseFloor number Minimum tilt to produce thrust (radians)
--- @return number effectiveTilt Filtered tilt magnitude
function HeliFilters.applyNoiseFloor(totalTiltRad, noiseFloor)
    if totalTiltRad < noiseFloor then
        return 0
    elseif totalTiltRad < noiseFloor * 2 then
        local rampFactor = (totalTiltRad - noiseFloor) / noiseFloor
        return totalTiltRad * rampFactor
    else
        return totalTiltRad
    end
end

-------------------------------------------------------------------------------------
-- Thrust decomposition: convert tilt deviations into world-space velocity.
-- Uses forward/right direction vectors and tilt fractions.
-------------------------------------------------------------------------------------
--- @param pitchDev number Effective pitch deviation (radians, 0 if blocked)
--- @param rollDev number Effective roll deviation (radians, 0 if blocked)
--- @param totalTiltRad number Raw combined tilt magnitude (for fraction computation)
--- @param fwdX number Forward direction X (PZ world space)
--- @param fwdZ number Forward direction Z (PZ world space)
--- @param maxHSpeed number Maximum horizontal speed (m/s)
--- @param effectiveTilt number Noise-floor-filtered tilt (from applyNoiseFloor)
--- @return number velX, number velZ, number totalSpeed
function HeliFilters.decomposeThrustDirection(pitchDev, rollDev, totalTiltRad, fwdX, fwdZ, maxHSpeed, effectiveTilt)
    local angle_90 = math.rad(90)
    local thrustFraction = math.sin(math.min(effectiveTilt, angle_90))
    local totalSpeed = thrustFraction * maxHSpeed

    local totalVelX = 0
    local totalVelZ = 0

    if totalTiltRad > 0.001 then
        -- Right direction (perpendicular to forward in 2D)
        local rightX = fwdZ
        local rightZ = -fwdX

        local pitchFraction = pitchDev / totalTiltRad
        local pitchSign = pitchDev > 0 and 1 or -1
        local rollFraction = rollDev / totalTiltRad
        local rollSign = rollDev > 0 and 1 or -1

        totalVelX = fwdX * math.abs(pitchFraction) * pitchSign * totalSpeed
                  + rightX * math.abs(rollFraction) * rollSign * totalSpeed
        totalVelZ = fwdZ * math.abs(pitchFraction) * pitchSign * totalSpeed
                  + rightZ * math.abs(rollFraction) * rollSign * totalSpeed
    end

    return totalVelX, totalVelZ, totalSpeed
end

-------------------------------------------------------------------------------------
-- Speed clamp: re-derive speed from vector and cap to maxHSpeed.
-------------------------------------------------------------------------------------
--- @param velX number Velocity X
--- @param velZ number Velocity Z
--- @param maxHSpeed number Maximum horizontal speed
--- @return number velX, number velZ, number totalSpeed (clamped)
function HeliFilters.clampSpeed(velX, velZ, maxHSpeed)
    local totalSpeed = math.sqrt(velX * velX + velZ * velZ)
    if totalSpeed > maxHSpeed then
        local scale = maxHSpeed / totalSpeed
        velX = velX * scale
        velZ = velZ * scale
        totalSpeed = maxHSpeed
    end
    return velX, velZ, totalSpeed
end

-------------------------------------------------------------------------------------
-- Thrust direction clamping: limit how far thrust can lead movement direction.
-- Skip for direction reversals (>90°).
-------------------------------------------------------------------------------------
--- @param velX number Thrust velocity X
--- @param velZ number Thrust velocity Z
--- @param totalSpeed number Thrust speed magnitude
--- @param moveAngle number Movement direction angle (radians, from atan2)
--- @param moveMag number Movement distance magnitude
--- @param maxLeadDeg number Maximum lead angle (degrees)
--- @return number velX, number velZ
function HeliFilters.clampThrustDirection(velX, velZ, totalSpeed, moveAngle, moveMag, maxLeadDeg)
    if totalSpeed <= 0.1 or moveMag <= 0.08 then
        return velX, velZ
    end

    local thrustAngle = math.atan2(velZ, velX)
    local angleDiff = thrustAngle - moveAngle
    if angleDiff > math.pi then angleDiff = angleDiff - 2 * math.pi end
    if angleDiff < -math.pi then angleDiff = angleDiff + 2 * math.pi end

    local maxLead = math.rad(maxLeadDeg)
    if math.abs(angleDiff) > maxLead and math.abs(angleDiff) < math.rad(90) then
        local clampedAngle = moveAngle + maxLead * (angleDiff > 0 and 1 or -1)
        velX = math.cos(clampedAngle) * totalSpeed
        velZ = math.sin(clampedAngle) * totalSpeed
    end

    return velX, velZ
end
