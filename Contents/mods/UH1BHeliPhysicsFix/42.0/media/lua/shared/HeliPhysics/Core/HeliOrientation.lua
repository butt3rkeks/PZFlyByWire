--[[
    HeliOrientation — Quaternion-state orientation for helicopter flight

    Separated yaw (scalar) + tilt (quaternion):
      _yawDeg  = heading in degrees (PZ convention, no gimbal ambiguity)
      _tiltQuat = pitch/roll quaternion (always near identity, clean matrix extraction)
      Combined: yawQ * _tiltQuat = full orientation for setAngles output

    Body-frame corrections:
      Pitch (ax): _tiltQuat = _tiltQuat * nqx (right-multiply)
      Roll (az):  _tiltQuat = _tiltQuat * nqz (right-multiply)
      Yaw (ay):   _yawDeg += ay (scalar, no quaternion extraction needed)

    Pure math + state. No keys, no vehicle, no HeliList, no game API.
]]

HeliOrientation = {}

-------------------------------------------------------------------------------------
-- CustomQuaternion library (pure Lua)
-------------------------------------------------------------------------------------
local CustomQuaternion = {}
CustomQuaternion.__index = CustomQuaternion

CustomQuaternion.__mul = function(q, r)
    local result = CustomQuaternion:new()
    local qx, qy, qz, qw = q:unpack()
    local rx, ry, rz, rw = r:unpack()
    result.x = qx * rw + qw * rx + qy * rz - qz * ry
    result.y = qy * rw + qw * ry + qz * rx - qx * rz
    result.z = qz * rw + qw * rz + qx * ry - qy * rx
    result.w = qw * rw - qx * rx - qy * ry - qz * rz
    return result
end

function CustomQuaternion:new()
    local o = { x = 0, y = 0, z = 0, w = 0 }
    setmetatable(o, CustomQuaternion)
    self.__index = self
    return o
end

CustomQuaternion.unpack = function(q) return q.x, q.y, q.z, q.w end

function CustomQuaternion.fromAngleAxis(angle, x, y, z)
    return CustomQuaternion:new():setAngleAxis(angle, x, y, z)
end

CustomQuaternion.setAngleAxis = function(q, angle, x, y, z)
    local s = math.sin(angle * .5)
    local c = math.cos(angle * .5)
    q.x = x * s; q.y = y * s; q.z = z * s; q.w = c
    return q
end

CustomQuaternion.length = function(q)
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
end

CustomQuaternion.normalize = function(q)
    local l = q:length()
    if l == 0 then q.x = 0; q.y = 0; q.z = 0; q.w = 1
    else l = 1 / l; q.x = q.x * l; q.y = q.y * l; q.z = q.z * l; q.w = q.w * l end
    return q
end

CustomQuaternion.toRotationMatrix = function(q)
    local q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    return 2*(q0*q0+q1*q1)-1, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2),
           2*(q1*q2+q0*q3), 2*(q0*q0+q2*q2)-1, 2*(q2*q3-q0*q1),
           2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 2*(q0*q0+q3*q3)-1
end

local function clamp(v, min, max)
    if v < min then return min end
    if v > max then return max end
    return v
end

CustomQuaternion.toEulerAngles = function(m11, m12, m13, m21, m22, m23, m31, m32, m33)
    local y = math.asin(clamp(m13, -1, 1))
    local x, z
    if math.abs(m13) < 0.9999999 then
        x = math.atan2(-m23, m33); z = math.atan2(-m12, m11)
    else
        x = math.atan2(m32, m22); z = 0
    end
    return math.deg(x), math.deg(y), math.deg(z)
end

CustomQuaternion.fromEulerAngles = function(x, y, z)
    local c1, c2, c3 = math.cos(x/2), math.cos(y/2), math.cos(z/2)
    local s1, s2, s3 = math.sin(x/2), math.sin(y/2), math.sin(z/2)
    local resQ = CustomQuaternion:new()
    resQ.x = s1*c2*c3 + c1*s2*s3
    resQ.y = c1*s2*c3 - s1*c2*s3
    resQ.z = c1*c2*s3 + s1*s2*c3
    resQ.w = c1*c2*c3 - s1*s2*s3
    return resQ
end

CustomQuaternion.conjugate = function(q)
    local r = CustomQuaternion:new()
    r.x = -q.x; r.y = -q.y; r.z = -q.z; r.w = q.w
    return r
end

-- Expose for external use (HeliInputProcessor needs fromAngleAxis for tilt application)
HeliOrientation.CustomQuaternion = CustomQuaternion

-------------------------------------------------------------------------------------
-- Orientation state: separated yaw (scalar) + tilt (quaternion)
-------------------------------------------------------------------------------------
local _yawDeg = nil
local _tiltQuat = nil

-------------------------------------------------------------------------------------
-- Compose full orientation from yaw + tilt for matrix extraction and setAngles.
-------------------------------------------------------------------------------------
local function composeOrientation()
    local yawQ = CustomQuaternion.fromAngleAxis(math.rad(_yawDeg), 0, 1, 0)
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
function HeliOrientation.initFromVehicle(angleX, angleY, angleZ)
    local fullQ = CustomQuaternion.fromEulerAngles(
        math.rad(angleX), math.rad(angleY), math.rad(angleZ))

    -- Extract yaw via forward vector (physical nose direction on ground plane).
    -- DO NOT use angleY — PZ may report non-canonical Euler Y (e.g., 12°)
    -- while the quaternion's physical heading is 168°.
    local fwd_x = 2 * (fullQ.x * fullQ.z + fullQ.w * fullQ.y)
    local fwd_z = 1 - 2 * (fullQ.x * fullQ.x + fullQ.y * fullQ.y)
    _yawDeg = math.deg(math.atan2(fwd_x, fwd_z))

    -- Identity quaternion = perfectly level
    _tiltQuat = CustomQuaternion:new()
    _tiltQuat.w = 1
end

--- Reset orientation state (re-init next frame).
function HeliOrientation.reset()
    _yawDeg = nil
    _tiltQuat = nil
end

--- Apply body-frame tilt corrections (degrees). Right-multiplied onto _tiltQuat.
--- @param ax number Pitch delta (degrees)
--- @param az number Roll delta (degrees)
function HeliOrientation.applyTilt(ax, az)
    if ax ~= 0 or az ~= 0 then
        local nqx = CustomQuaternion.fromAngleAxis(math.rad(ax), 1, 0, 0)
        local nqz = CustomQuaternion.fromAngleAxis(math.rad(az), 0, 0, 1)
        _tiltQuat = _tiltQuat * nqx * nqz
        _tiltQuat:normalize()
    end
end

--- Apply yaw delta (degrees). Scalar addition, no quaternion extraction.
--- @param ay number Yaw delta (degrees)
function HeliOrientation.applyYaw(ay)
    _yawDeg = _yawDeg + ay
end

--- Get body-frame pitch from _tiltQuat (heading-independent).
--- @return number Pitch angle in radians (acos(R12) from tilt matrix)
function HeliOrientation.getBodyPitch()
    local _, _, _, _, _, tr12, _, _, _ = CustomQuaternion.toRotationMatrix(_tiltQuat)
    return math.acos(clamp(tr12, -1, 1))
end

--- Get body-frame roll from _tiltQuat (heading-independent).
--- @return number Roll angle in radians (acos(R10) from tilt matrix)
function HeliOrientation.getBodyRoll()
    local _, _, _, tr10, _, _, _, _, _ = CustomQuaternion.toRotationMatrix(_tiltQuat)
    return math.acos(clamp(tr10, -1, 1))
end

--- Get forward direction from full composed orientation.
--- @return number fwdPzX, number fwdPzY Forward direction in PZ world space
function HeliOrientation.getForward()
    local fullQ = composeOrientation()
    local r00, r01, r02, r10, r11, r12, r20, r21, r22 = CustomQuaternion.toRotationMatrix(fullQ)
    return r02, r22
end

--- Get current yaw in degrees.
--- @return number Yaw in degrees
function HeliOrientation.getYaw()
    return _yawDeg
end

--- Set yaw to exact value (scalar assignment, trivial hard lock).
--- @param deg number Yaw in degrees
function HeliOrientation.setYaw(deg)
    _yawDeg = deg
end

--- Convert full orientation to Euler angles for setAngles output.
--- @return number exDeg, number eyDeg, number ezDeg Euler angles in degrees
function HeliOrientation.toEuler()
    local fullQ = composeOrientation()
    return CustomQuaternion.toEulerAngles(CustomQuaternion.toRotationMatrix(fullQ))
end

--- Check if orientation has been initialized.
--- @return boolean
function HeliOrientation.isInitialized()
    return _yawDeg ~= nil
end
