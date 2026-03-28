--[[
    TRQCoupleForce — Convert torque vector into couple-force impulse calls

    Two opposing applyImpulseGeneric calls per axis: linear forces cancel,
    torques add. Up to 6 calls per frame (2 per active axis).

    Torque input and all internal math use standard math convention (Y = up).
    PZ coordinate swap (Y↔Z) applied at the applyImpulseGeneric boundary.

    Cross-product verification (standard math, Y=up):
      Pitch (X): r=(0,0,+d) × F=(0,-f,0) = (+df, 0, 0) ✓
      Yaw   (Y): r=(+d,0,0) × F=(0,0,-f) = (0, +df, 0) ✓
      Roll  (Z): r=(0,+d,0) × F=(-f,0,0) = (0, 0, +df) ✓

    Coordinate swap for applyImpulseGeneric (PZ convention):
      Standard (sx, sy, sz) → PZ from: (vx+sx, vy+sz, vz+sy)
      Standard (fx, fy, fz) → PZ dir:  (fx, fz, fy)
    Java then swaps back internally (BaseVehicle.java:5541-5544).
]]

TRQCoupleForce = {}

local abs = math.abs
local sqrt = math.sqrt
local toLuaNum = HeliUtil.toLuaNum

local MIN_TORQUE = 0.001
local IMPULSE_GENERIC_MULTIPLIER = 30.0

-------------------------------------------------------------------------------------
-- Low-level: apply one force at an offset (both in standard math convention)
-------------------------------------------------------------------------------------

--- Apply a single force at an offset from vehicle center.
--- All parameters in standard math convention (Y = up).
--- @param vehicle BaseVehicle
--- @param vx number Vehicle PZ X
--- @param vy number Vehicle PZ Y (north)
--- @param vz number Vehicle PZ Z (up/height)
--- @param offX number Standard math offset X
--- @param offY number Standard math offset Y (up)
--- @param offZ number Standard math offset Z (north)
--- @param forceX number Standard math force X
--- @param forceY number Standard math force Y (up)
--- @param forceZ number Standard math force Z (north)
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
    --   from: (vx + offX, vy + offZ, vz + offY)  [PZ: X, Y=north=stdZ, Z=up=stdY]
    --   dir:  (dirX, dirZ, dirY)                  [PZ: X, Y=north=stdZ, Z=up=stdY]
    vehicle:applyImpulseGeneric(
        vx + offX, vy + offZ, vz + offY,
        dirX, dirZ, dirY,
        strength
    )
end

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Apply couple forces to produce pure torque (zero net linear force).
--- Torque vector in standard math convention (Y = up).
--- @param vehicle BaseVehicle
--- @param torqueX number Torque around X (pitch)
--- @param torqueY number Torque around Y (yaw)
--- @param torqueZ number Torque around Z (roll)
function TRQCoupleForce.apply(vehicle, torqueX, torqueY, torqueZ)
    local d = HeliConfig.GetTrqCoupleOffset()
    if d <= 0 then return end

    local vx = toLuaNum(vehicle:getX())
    local vy = toLuaNum(vehicle:getY())
    local vz = toLuaNum(vehicle:getZ())

    -- Pitch (torque around X): offset along Z, force along Y
    if abs(torqueX) > MIN_TORQUE then
        local f = torqueX / (2 * d)
        -- Call 1: offset (0, 0, +d), force (0, -f, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0, 0,  d,  0, -f, 0)
        -- Call 2: offset (0, 0, -d), force (0, +f, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0, 0, -d,  0,  f, 0)
    end

    -- Yaw (torque around Y): offset along X, force along Z
    if abs(torqueY) > MIN_TORQUE then
        local f = torqueY / (2 * d)
        -- Call 1: offset (+d, 0, 0), force (0, 0, -f)
        applyOffsetForce(vehicle, vx, vy, vz,  d, 0, 0,  0, 0, -f)
        -- Call 2: offset (-d, 0, 0), force (0, 0, +f)
        applyOffsetForce(vehicle, vx, vy, vz, -d, 0, 0,  0, 0,  f)
    end

    -- Roll (torque around Z): offset along Y, force along X
    if abs(torqueZ) > MIN_TORQUE then
        local f = torqueZ / (2 * d)
        -- Call 1: offset (0, +d, 0), force (-f, 0, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0, d, 0,  -f, 0, 0)
        -- Call 2: offset (0, -d, 0), force (+f, 0, 0)
        applyOffsetForce(vehicle, vx, vy, vz,  0,-d, 0,   f, 0, 0)
    end
end
