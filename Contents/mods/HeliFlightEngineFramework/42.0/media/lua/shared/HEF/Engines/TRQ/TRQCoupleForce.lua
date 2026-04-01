--[[
    TRQCoupleForce — Apply body-frame torque via body-aligned couple forces

    Two opposing applyImpulseGeneric calls per axis: linear forces cancel,
    torques add. Up to 6 calls per frame (2 per active axis).

    Body-aligned: couple force offsets and directions follow the vehicle's
    body axes (right, up, forward), not world axes. This means the torque
    IS applied around body axes, so body-frame inertia (Ix for pitch,
    Iz for roll) is physically correct at any heading.

    The body axes are passed as world-coordinate vectors (R matrix columns).

    Couple geometry (body frame):
      Pitch (body X): offset along body-forward, force along body-up
        body_fwd × body_up = body_right (perpendicular) → torque around body-X ✓
      Yaw (body Y):   offset along body-right, force along body-forward
        body_right × body_fwd has Y component → torque around body-Y ✓
      Roll (body Z):  offset along body-up, force along body-right
        body_up × body_right = body_fwd (perpendicular) → torque around body-Z ✓
]]

TRQCoupleForce = {}

local abs = math.abs
local sqrt = math.sqrt
local toLuaNum = HeliUtil.toLuaNum

local MIN_TORQUE = 0.001
local IMPULSE_GENERIC_MULTIPLIER = 30.0

--- Apply a single force at an offset from vehicle center.
--- All parameters in world coordinates.
local function applyOffsetForce(vehicle, vx, vy, vz,
                                offX, offY, offZ,
                                forceX, forceY, forceZ)
    local mag = sqrt(forceX*forceX + forceY*forceY + forceZ*forceZ)
    if mag < MIN_TORQUE then return end

    local strength = mag / IMPULSE_GENERIC_MULTIPLIER
    local dirX = forceX / mag
    local dirY = forceY / mag
    local dirZ = forceZ / mag

    -- Standard math → PZ coordinate swap:
    --   from: (vx + offX, vy + offZ, vz + offY)
    --   dir:  (dirX, dirZ, dirY)
    vehicle:applyImpulseGeneric(
        vx + offX, vy + offZ, vz + offY,
        dirX, dirZ, dirY,
        strength
    )
end

--- Apply world-frame torque via world-axis-fixed couple forces.
--- Standard math convention (Y = up). Works at any tilt with uniform inertia.
--- @param vehicle BaseVehicle
--- @param torqueX number Torque around world X (Nm)
--- @param torqueY number Torque around world Y (Nm)
--- @param torqueZ number Torque around world Z (Nm)
function TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)
    local d = HeliConfig.GetTrqCoupleOffset()
    if d <= 0 then return end

    local vx = toLuaNum(vehicle:getX())
    local vy = toLuaNum(vehicle:getY())
    local vz = toLuaNum(vehicle:getZ())

    -- Pitch (torque around X): offset along Z, force along Y
    if abs(torqueX) > MIN_TORQUE then
        local f = torqueX / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,  0, 0,  d,  0, -f, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0, 0, -d,  0,  f, 0)
    end

    -- Yaw (torque around Y): offset along X, force along Z
    if abs(torqueY) > MIN_TORQUE then
        local f = torqueY / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,  d, 0, 0,  0, 0, -f)
        applyOffsetForce(vehicle, vx, vy, vz, -d, 0, 0,  0, 0,  f)
    end

    -- Roll (torque around Z): offset along Y, force along X
    if abs(torqueZ) > MIN_TORQUE then
        local f = torqueZ / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,  0, d, 0,  -f, 0, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0,-d, 0,   f, 0, 0)
    end
end

--- Apply body-frame torque via body-aligned couple forces.
--- bodyTorqueX = pitch, bodyTorqueY = yaw, bodyTorqueZ = roll (body frame Nm).
--- Body axes are world-coordinate vectors (columns of rotation matrix R).
--- @param vehicle BaseVehicle
--- @param bodyTorqueX number Body pitch torque (Nm)
--- @param bodyTorqueY number Body yaw torque (Nm)
--- @param bodyTorqueZ number Body roll torque (Nm)
--- @param rightX number Body-right (X) axis, world X component
--- @param rightY number Body-right (X) axis, world Y component
--- @param rightZ number Body-right (X) axis, world Z component
--- @param upX number Body-up (Y) axis, world X component
--- @param upY number Body-up (Y) axis, world Y component
--- @param upZ number Body-up (Y) axis, world Z component
--- @param fwdX number Body-forward (Z) axis, world X component
--- @param fwdY number Body-forward (Z) axis, world Y component
--- @param fwdZ number Body-forward (Z) axis, world Z component
function TRQCoupleForce.applyBodyAligned(vehicle,
                                         bodyTorqueX, bodyTorqueY, bodyTorqueZ,
                                         rightX, rightY, rightZ,
                                         upX, upY, upZ,
                                         fwdX, fwdY, fwdZ)
    local d = HeliConfig.GetTrqCoupleOffset()
    if d <= 0 then return end

    local vx = toLuaNum(vehicle:getX())
    local vy = toLuaNum(vehicle:getY())
    local vz = toLuaNum(vehicle:getZ())

    -- Pitch (body X torque): offset along body-forward, force along body-up
    -- Cross: body_fwd × body_up = body_right direction → torque around body-X
    if abs(bodyTorqueX) > MIN_TORQUE then
        local f = bodyTorqueX / (2 * d)
        -- Call 1: offset +d*forward, force -f*up
        applyOffsetForce(vehicle, vx, vy, vz,
            d*fwdX, d*fwdY, d*fwdZ,
            -f*upX, -f*upY, -f*upZ)
        -- Call 2: offset -d*forward, force +f*up
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*fwdX, -d*fwdY, -d*fwdZ,
            f*upX, f*upY, f*upZ)
    end

    -- Yaw (body Y torque): offset along body-right, force along body-forward
    -- Cross: body_right × body_fwd has body-up component → torque around body-Y
    if abs(bodyTorqueY) > MIN_TORQUE then
        local f = bodyTorqueY / (2 * d)
        -- Call 1: offset +d*right, force -f*forward
        applyOffsetForce(vehicle, vx, vy, vz,
            d*rightX, d*rightY, d*rightZ,
            -f*fwdX, -f*fwdY, -f*fwdZ)
        -- Call 2: offset -d*right, force +f*forward
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*rightX, -d*rightY, -d*rightZ,
            f*fwdX, f*fwdY, f*fwdZ)
    end

    -- Roll (body Z torque): offset along body-up, force along body-right
    -- Cross: body_up × body_right = body_fwd direction → torque around body-Z
    if abs(bodyTorqueZ) > MIN_TORQUE then
        local f = bodyTorqueZ / (2 * d)
        -- Call 1: offset +d*up, force -f*right
        applyOffsetForce(vehicle, vx, vy, vz,
            d*upX, d*upY, d*upZ,
            -f*rightX, -f*rightY, -f*rightZ)
        -- Call 2: offset -d*up, force +f*right
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*upX, -d*upY, -d*upZ,
            f*rightX, f*rightY, f*rightZ)
    end
end
