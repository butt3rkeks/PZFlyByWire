--[[
    ADRCCoupleForce -- Apply body-frame torque via body-aligned couple forces

    Two opposing applyImpulseGeneric calls per axis: linear forces cancel,
    torques add. Up to 6 calls per frame (2 per active axis).

    Body-aligned: couple force offsets and directions follow the vehicle's
    body axes (right, up, forward), so torque IS applied around body axes
    at any heading. Body-frame inertia (Ix for pitch, Iz for roll) is
    physically correct regardless of orientation.

    Verified force chain (from MEMORY.md):
      ctx.applyForce -> HeliForceAdapter -> applyImpulseGeneric -> Java computes
      relPos and torque from (applicationPoint - CoM). Forces at vehicle center
      produce zero torque. Couple forces use intentional offset for pure torque.

    Couple geometry (body frame):
      Pitch (body X): offset along body-forward, force along body-up
      Yaw (body Y):   offset along body-right, force along body-forward
      Roll (body Z):  offset along body-up, force along body-right

    PZ coordinate swap: applyImpulseGeneric expects (x, z, y) for position
    and (dirX, dirZ, dirY) for direction. The applyOffsetForce helper handles
    this swap transparently.

    IMPULSE_GENERIC_MULTIPLIER = 30: applyImpulseGeneric interprets strength as
    force/30, so we divide by 30 to get the intended Newton force.
]]

ADRCCoupleForce = {}

local abs = math.abs
local sqrt = math.sqrt
local toLuaNum = HeliUtil.toLuaNum

local MIN_TORQUE = 0.001
local IMPULSE_GENERIC_MULTIPLIER = 30.0

--- Apply a single force at an offset from vehicle center.
--- All parameters in standard math coordinates (X=east, Y=up, Z=north).
--- Handles the PZ Y/Z swap internally.
--- @param vehicle BaseVehicle
--- @param vx number Vehicle world position X
--- @param vy number Vehicle world position Y (PZ vertical)
--- @param vz number Vehicle world position Z (PZ horizontal)
--- @param offX number Offset from vehicle center X (standard math)
--- @param offY number Offset from vehicle center Y (standard math, up)
--- @param offZ number Offset from vehicle center Z (standard math)
--- @param forceX number Force direction X (standard math)
--- @param forceY number Force direction Y (standard math, up)
--- @param forceZ number Force direction Z (standard math)
local function applyOffsetForce(vehicle, vx, vy, vz,
                                offX, offY, offZ,
                                forceX, forceY, forceZ)
    local mag = sqrt(forceX*forceX + forceY*forceY + forceZ*forceZ)
    if mag < MIN_TORQUE then return end

    local strength = mag / IMPULSE_GENERIC_MULTIPLIER
    local dirX = forceX / mag
    local dirY = forceY / mag
    local dirZ = forceZ / mag

    -- Standard math -> PZ coordinate swap:
    --   position: (vx + offX, vy + offZ, vz + offY)
    --   direction: (dirX, dirZ, dirY)
    vehicle:applyImpulseGeneric(
        vx + offX, vy + offZ, vz + offY,
        dirX, dirZ, dirY,
        strength
    )
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
function ADRCCoupleForce.applyBodyAligned(vehicle,
                                          bodyTorqueX, bodyTorqueY, bodyTorqueZ,
                                          rightX, rightY, rightZ,
                                          upX, upY, upZ,
                                          fwdX, fwdY, fwdZ)
    local d = HeliConfig.GetAdrcCoupleOffset()
    if d <= 0 then return end

    local vx = toLuaNum(vehicle:getX())
    local vy = toLuaNum(vehicle:getY())
    local vz = toLuaNum(vehicle:getZ())

    -- Pitch (body X torque): offset along body-forward, force along body-up
    -- Cross: body_fwd x body_up = body_right -> torque around body-X
    if abs(bodyTorqueX) > MIN_TORQUE then
        local f = bodyTorqueX / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,
            d*fwdX, d*fwdY, d*fwdZ,
            -f*upX, -f*upY, -f*upZ)
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*fwdX, -d*fwdY, -d*fwdZ,
            f*upX, f*upY, f*upZ)
    end

    -- Yaw (body Y torque): offset along body-right, force along body-forward
    -- Cross: body_right x body_fwd has body-up component -> torque around body-Y
    if abs(bodyTorqueY) > MIN_TORQUE then
        local f = bodyTorqueY / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,
            d*rightX, d*rightY, d*rightZ,
            -f*fwdX, -f*fwdY, -f*fwdZ)
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*rightX, -d*rightY, -d*rightZ,
            f*fwdX, f*fwdY, f*fwdZ)
    end

    -- Roll (body Z torque): offset along body-up, force along body-right
    -- Cross: body_up x body_right = body_fwd -> torque around body-Z
    if abs(bodyTorqueZ) > MIN_TORQUE then
        local f = bodyTorqueZ / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,
            d*upX, d*upY, d*upZ,
            -f*rightX, -f*rightY, -f*rightZ)
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*upX, -d*upY, -d*upZ,
            f*rightX, f*rightY, f*rightZ)
    end
end
