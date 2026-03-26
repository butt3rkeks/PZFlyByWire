--[[
    FBWOrientation — Quaternion-state orientation for FBW flight engine

    Separated yaw (scalar) + tilt (quaternion):
      _yawDeg  = heading in degrees (PZ convention, no gimbal ambiguity)
      _tiltQuat = pitch/roll quaternion (always near identity, clean matrix extraction)
      Combined: yawQ * _tiltQuat = full orientation for setAngles output

    Body-frame corrections:
      Pitch (ax): _tiltQuat = _tiltQuat * nqx (right-multiply)
      Roll (az):  _tiltQuat = _tiltQuat * nqz (right-multiply)
      Yaw (ay):   _yawDeg += ay (scalar, no quaternion extraction needed)

    Uses Models/Quaternion for all quaternion math.
    Pure math + state. No keys, no vehicle, no HeliList, no game API.
]]

FBWOrientation = {}

local function clamp(v, min, max)
    if v < min then return min end
    if v > max then return max end
    return v
end

-------------------------------------------------------------------------------------
-- Orientation state: separated yaw (scalar) + tilt (quaternion)
-------------------------------------------------------------------------------------
local _yawDeg = nil
local _tiltQuat = nil

-------------------------------------------------------------------------------------
-- Compose full orientation from yaw + tilt for matrix extraction and setAngles.
-------------------------------------------------------------------------------------
local function composeOrientation()
    local yawQ = Quaternion.fromAxisAngle(math.rad(_yawDeg), 0, 1, 0)
    return yawQ * _tiltQuat
end

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Initialize from vehicle's Euler angles (first frame only).
--- Extracts yaw via forward vector projection (branch-unambiguous).
--- Sets tilt to identity (level). DO NOT capture Bullet's ground-state tilt —
--- suspension/slope settling causes violent lurch on takeoff.
--- @param angleX number Vehicle Euler X (degrees)
--- @param angleY number Vehicle Euler Y (degrees)
--- @param angleZ number Vehicle Euler Z (degrees)
function FBWOrientation.initFromVehicle(angleX, angleY, angleZ)
    local fullQ = Quaternion.fromEuler(
        math.rad(angleX), math.rad(angleY), math.rad(angleZ))

    -- Extract yaw via forward vector (physical nose direction on ground plane).
    -- DO NOT use angleY — PZ may report non-canonical Euler Y (e.g., 12°)
    -- while the quaternion's physical heading is 168°.
    local fwd_x = 2 * (fullQ.x * fullQ.z + fullQ.w * fullQ.y)
    local fwd_z = 1 - 2 * (fullQ.x * fullQ.x + fullQ.y * fullQ.y)
    _yawDeg = math.deg(math.atan2(fwd_x, fwd_z))

    -- Identity quaternion = perfectly level
    _tiltQuat = Quaternion.identity()
end

--- Reset orientation state (re-init next frame).
function FBWOrientation.reset()
    _yawDeg = nil
    _tiltQuat = nil
end

--- Apply body-frame tilt corrections (degrees). Right-multiplied onto _tiltQuat.
--- @param ax number Pitch delta (degrees)
--- @param az number Roll delta (degrees)
function FBWOrientation.applyTilt(ax, az)
    if ax ~= 0 or az ~= 0 then
        local nqx = Quaternion.fromAxisAngle(math.rad(ax), 1, 0, 0)
        local nqz = Quaternion.fromAxisAngle(math.rad(az), 0, 0, 1)
        _tiltQuat = _tiltQuat * nqx * nqz
        _tiltQuat:normalize()
    end
end

--- Apply yaw delta (degrees). Scalar addition, no quaternion extraction.
--- @param ay number Yaw delta (degrees)
function FBWOrientation.applyYaw(ay)
    _yawDeg = _yawDeg + ay
end

--- Get body-frame pitch from _tiltQuat (heading-independent).
--- @return number Pitch angle in radians (acos(R12) from tilt matrix)
function FBWOrientation.getBodyPitch()
    local _, _, _, _, _, tr12, _, _, _ = _tiltQuat:toMatrixComponents()
    return math.acos(clamp(tr12, -1, 1))
end

--- Get body-frame roll from _tiltQuat (heading-independent).
--- @return number Roll angle in radians (acos(R10) from tilt matrix)
function FBWOrientation.getBodyRoll()
    local _, _, _, tr10, _, _, _, _, _ = _tiltQuat:toMatrixComponents()
    return math.acos(clamp(tr10, -1, 1))
end

--- Get forward direction from full composed orientation.
--- @return number fwdPzX, number fwdPzY Forward direction in PZ world space
function FBWOrientation.getForward()
    local fullQ = composeOrientation()
    local r00, r01, r02, r10, r11, r12, r20, r21, r22 = fullQ:toMatrixComponents()
    return r02, r22
end

--- Get current yaw in degrees.
--- @return number Yaw in degrees
function FBWOrientation.getYaw()
    return _yawDeg
end

--- Set yaw to exact value (scalar assignment, trivial hard lock).
--- @param deg number Yaw in degrees
function FBWOrientation.setYaw(deg)
    _yawDeg = deg
end

--- Convert full orientation to Euler angles for setAngles output.
--- @return number exDeg, number eyDeg, number ezDeg Euler angles in degrees
function FBWOrientation.toEuler()
    local fullQ = composeOrientation()
    return Quaternion.matrixToEuler(fullQ:toMatrixComponents())
end

--- Check if orientation has been initialized.
--- @return boolean
function FBWOrientation.isInitialized()
    return _yawDeg ~= nil
end
