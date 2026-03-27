--[[
    VelocityUtil — Common velocity decomposition helpers

    Eliminates repeated inline math.sqrt(vx*vx + vz*vz) across engine code.
    All functions are stateless — pure math on input values.
]]

VelocityUtil = {}

--- Horizontal speed (ground plane, ignoring vertical).
--- @param velX number Horizontal velocity X
--- @param velZ number Horizontal velocity Z
--- @return number Horizontal speed (always >= 0)
function VelocityUtil.horizontalSpeed(velX, velZ)
    return math.sqrt(velX * velX + velZ * velZ)
end

--- Total speed (all three axes).
--- @param velX number
--- @param velY number
--- @param velZ number
--- @return number Total speed (always >= 0)
function VelocityUtil.totalSpeed(velX, velY, velZ)
    return math.sqrt(velX * velX + velY * velY + velZ * velZ)
end

--- Horizontal direction angle (radians, atan2 convention).
--- Returns 0 when stationary (magnitude below threshold).
--- @param velX number Horizontal velocity X
--- @param velZ number Horizontal velocity Z
--- @param threshold number Minimum magnitude to compute angle (default 0.01)
--- @return number Angle in radians, number magnitude
function VelocityUtil.horizontalAngle(velX, velZ, threshold)
    threshold = threshold or 0.01
    local mag = math.sqrt(velX * velX + velZ * velZ)
    if mag < threshold then return 0, mag end
    return math.atan2(velZ, velX), mag
end

--- Decompose velocity into horizontal and vertical components.
--- @param velX number
--- @param velY number
--- @param velZ number
--- @return number hSpeed, number vSpeed (signed, positive = up)
function VelocityUtil.decompose(velX, velY, velZ)
    return math.sqrt(velX * velX + velZ * velZ), velY
end
