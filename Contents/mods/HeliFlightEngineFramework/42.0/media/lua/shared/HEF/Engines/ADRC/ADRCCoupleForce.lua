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

--- Apply world-frame torque vector via world-axis couple forces.
--- Used for single-axis tilt correction (no body decomposition).
--- The torque vector (tiltX, tiltY, tiltZ) is in world/standard math coords.
--- Yaw torque is applied separately via body-up couple.
--- @param vehicle BaseVehicle
--- @param tiltTorqueX number World-frame torque X component (Nm)
--- @param tiltTorqueY number World-frame torque Y component (Nm, usually small)
--- @param tiltTorqueZ number World-frame torque Z component (Nm)
--- @param yawTorque number Yaw torque (Nm, applied around body-up axis)
--- @param upX number Body-up X, @param upY number, @param upZ number
--- @param rightX number Body-right X, @param rightY number, @param rightZ number
--- @param fwdX number Body-fwd X, @param fwdY number, @param fwdZ number
function ADRCCoupleForce.applyWorldTilt(vehicle,
                                        tiltTorqueX, tiltTorqueY, tiltTorqueZ,
                                        yawTorque,
                                        rightX, rightY, rightZ,
                                        upX, upY, upZ,
                                        fwdX, fwdY, fwdZ)
    local d = HeliConfig.GetAdrcCoupleOffset()
    if d <= 0 then return end

    local vx = toLuaNum(vehicle:getX())
    local vy = toLuaNum(vehicle:getY())
    local vz = toLuaNum(vehicle:getZ())

    -- World-frame tilt torque via world-axis couple forces.
    -- X torque: offset along Z, force along Y
    if abs(tiltTorqueX) > MIN_TORQUE then
        local f = tiltTorqueX / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz, 0, 0, d,  0, -f, 0)
        applyOffsetForce(vehicle, vx, vy, vz, 0, 0, -d, 0, f, 0)
    end

    -- Y torque (small, from tilt axis having a vertical component): offset along X, force along Z
    if abs(tiltTorqueY) > MIN_TORQUE then
        local f = tiltTorqueY / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz, d, 0, 0,  0, 0, -f)
        applyOffsetForce(vehicle, vx, vy, vz, -d, 0, 0, 0, 0, f)
    end

    -- Z torque: offset along Y, force along X
    if abs(tiltTorqueZ) > MIN_TORQUE then
        local f = tiltTorqueZ / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz, 0, d, 0,  -f, 0, 0)
        applyOffsetForce(vehicle, vx, vy, vz, 0, -d, 0, f, 0, 0)
    end

    -- Yaw: still body-aligned (around body-up axis)
    if abs(yawTorque) > MIN_TORQUE then
        local f = yawTorque / (2 * d)
        applyOffsetForce(vehicle, vx, vy, vz,
            d*rightX, d*rightY, d*rightZ,
            -f*fwdX, -f*fwdY, -f*fwdZ)
        applyOffsetForce(vehicle, vx, vy, vz,
            -d*rightX, -d*rightY, -d*rightZ,
            f*fwdX, f*fwdY, f*fwdZ)
    end
end
