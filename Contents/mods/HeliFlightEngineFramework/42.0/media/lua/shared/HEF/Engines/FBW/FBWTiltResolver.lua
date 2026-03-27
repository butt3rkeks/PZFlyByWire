--[[
    FBWTiltResolver — Resolve body tilt into desired horizontal velocity

    Orchestrates: wall pre-blocking → FBWFilters pipeline (noise floor →
    thrust decomposition → speed clamp → direction clamp).

    Owns _prevPos state for direction clamping. Reads HeliConfig for
    thresholds. This is the pipeline-sequencing layer *above* FBWFilters
    (which remains pure/stateless with all parameters passed in).

    Does NOT own: simulation model, error tracker, input flags, FA-off logic.
]]

FBWTiltResolver = {}

-------------------------------------------------------------------------------------
-- State: previous position for direction clamp (one frame lookback)
-------------------------------------------------------------------------------------
local _prevPosX, _prevPosZ = nil, nil

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset state for new flight session.
function FBWTiltResolver.reset()
    _prevPosX, _prevPosZ = nil, nil
end

--- Resolve body angles + wall blocking into a desired horizontal velocity vector.
--- Runs the full pipeline: wall pre-block → noise floor → thrust decomposition →
--- speed clamp → direction clamp.
---
--- @param angleZ number Body pitch angle (radians, from FBWOrientation.getBodyPitch)
--- @param angleX number Body roll angle (radians, from FBWOrientation.getBodyRoll)
--- @param blocked table Framework-provided blocked directions {up, down, left, right}
--- @param fwdX number Forward direction X (PZ world space)
--- @param fwdZ number Forward direction Z (PZ world space)
--- @param posX number Current world X position (for direction clamp delta)
--- @param posZ number Current world Z position (for direction clamp delta)
--- @return number velX Desired horizontal velocity X
--- @return number velZ Desired horizontal velocity Z
--- @return number speed Desired horizontal speed magnitude
--- @return number totalTiltRad Combined tilt magnitude after wall blocking (radians)
--- @return boolean isBlockedHit Whether any axis was wall-blocked this frame
function FBWTiltResolver.resolve(angleZ, angleX, blocked, fwdX, fwdZ, posX, posZ)
    -- Step 7: Wall pre-blocking — zero out tilt axes that push into walls
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

    -- Step 8: FBWFilters pipeline — noiseFloor → thrustDecomposition → speedClamp → directionClamp
    local noiseFloor = HeliConfig.TILT_NOISE_FLOOR
    local maxHSpeed  = HeliConfig.GetMaxHorizontalSpeed()

    local effectiveTilt = FBWFilters.applyNoiseFloor(totalTiltRad, noiseFloor)

    local velX, velZ, speed = FBWFilters.decomposeThrustDirection(
        pitchDev, rollDev, totalTiltRad, fwdX, fwdZ, maxHSpeed, effectiveTilt)

    velX, velZ, speed = FBWFilters.clampSpeed(velX, velZ, maxHSpeed)

    -- Direction clamp: limit how far thrust can lead actual movement direction.
    -- Uses previous frame's position to compute movement angle.
    if _prevPosX and speed > 0.1 then
        local dx = posX - _prevPosX
        local dz = posZ - _prevPosZ
        local moveMag = math.sqrt(dx * dx + dz * dz)
        if moveMag > 0.08 then
            local moveAngle = math.atan2(dz, dx)
            velX, velZ = FBWFilters.clampThrustDirection(
                velX, velZ, speed, moveAngle, moveMag, HeliConfig.MAX_THRUST_LEAD)
        end
    end
    _prevPosX, _prevPosZ = posX, posZ

    return velX, velZ, speed, totalTiltRad, isBlockedHit
end
