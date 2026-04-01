--[[
    TRQOrientation — Desired orientation tracker for the torque-based engine

    Torque-tailored design:
      - Desired tilt is a target the PD chases (same quaternion compose as FBW)
      - Body angles for InputProcessor come from ACTUAL vehicle state (Bullet),
        not from the desired orientation. With torque, there's always a lag
        between desired and actual — auto-level must respond to real tilt.
      - Forward direction for tilt resolver comes from actual vehicle.
      - Desired yaw tracked as scalar, no reanchor snap (via TRQYawController).

    The actual vehicle state is passed in via updateActualState() each frame,
    before InputProcessor reads body angles.

    Uses Models/Quaternion for desired quaternion math.
]]

TRQOrientation = {}

local function clamp(v, min, max)
    if v < min then return min end
    if v > max then return max end
    return v
end

-------------------------------------------------------------------------------------
-- Desired orientation state: separated yaw (scalar) + tilt (quaternion)
-------------------------------------------------------------------------------------
local _yawDeg = nil
local _tiltQuat = nil

-------------------------------------------------------------------------------------
-- Actual vehicle state (updated from Bullet each frame)
-------------------------------------------------------------------------------------
local _actUpX = 0
local _actUpY = 1
local _actUpZ = 0
local _actFwdX = 0
local _actFwdZ = 1
local _actYawDeg = 0

-------------------------------------------------------------------------------------
-- Compose full desired orientation from yaw + tilt.
-------------------------------------------------------------------------------------
local function composeOrientation()
    local yawQ = Quaternion.fromAxisAngle(math.rad(_yawDeg), 0, 1, 0)
    return yawQ * _tiltQuat
end

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Initialize from vehicle's Euler angles (first frame only).
--- @param angleX number Vehicle Euler X (degrees)
--- @param angleY number Vehicle Euler Y (degrees)
--- @param angleZ number Vehicle Euler Z (degrees)
function TRQOrientation.initFromVehicle(angleX, angleY, angleZ)
    local fullQuaternion = Quaternion.fromEuler(
        math.rad(angleX), math.rad(angleY), math.rad(angleZ))

    local forwardX = 2 * (fullQuaternion.x * fullQuaternion.z + fullQuaternion.w * fullQuaternion.y)
    local forwardZ = 1 - 2 * (fullQuaternion.x * fullQuaternion.x + fullQuaternion.y * fullQuaternion.y)
    _yawDeg = math.deg(math.atan2(forwardX, forwardZ))

    _tiltQuat = Quaternion.identity()
end

--- Update actual vehicle state from Bullet readings.
--- Must be called each frame BEFORE InputProcessor or tilt resolver.
--- @param upX number Actual up-vector X (world frame)
--- @param upY number Actual up-vector Y (world frame)
--- @param upZ number Actual up-vector Z (world frame)
--- @param fwdX number Actual forward X (world frame)
--- @param fwdZ number Actual forward Z (world frame)
--- @param yawDeg number Actual yaw (degrees, from atan2 of forward vector)
function TRQOrientation.updateActualState(upX, upY, upZ, fwdX, fwdZ, yawDeg)
    _actUpX = upX
    _actUpY = upY
    _actUpZ = upZ
    _actFwdX = fwdX
    _actFwdZ = fwdZ
    _actYawDeg = yawDeg
end

--- Reset orientation state.
function TRQOrientation.reset()
    _yawDeg = nil
    _tiltQuat = nil
    _actUpX = 0; _actUpY = 1; _actUpZ = 0
    _actFwdX = 0; _actFwdZ = 1
    _actYawDeg = 0
end

--- Apply body-frame tilt corrections to the desired orientation.
--- @param ax number Pitch delta (degrees)
--- @param az number Roll delta (degrees)
function TRQOrientation.applyTilt(ax, az)
    if ax ~= 0 or az ~= 0 then
        local nqx = Quaternion.fromAxisAngle(math.rad(ax), 1, 0, 0)
        local nqz = Quaternion.fromAxisAngle(math.rad(az), 0, 0, 1)
        _tiltQuat = _tiltQuat * nqx * nqz
        _tiltQuat:normalize()
    end
end

--- Decay desired tilt toward level when no directional input.
--- Uses SLERP toward identity quaternion at the given rate.
--- @param rate number Decay speed (0-1 per frame, 0.05 = ~1s to level)
function TRQOrientation.decayTiltToLevel(rate)
    if not _tiltQuat then return end
    -- SLERP toward identity: tiltQuat = lerp(tiltQuat, identity, rate)
    -- For small angles, lerp ≈ slerp. Identity = (1,0,0,0).
    local id = Quaternion.identity()
    local t = math.min(rate, 1.0)
    _tiltQuat.w = _tiltQuat.w + (id.w - _tiltQuat.w) * t
    _tiltQuat.x = _tiltQuat.x + (id.x - _tiltQuat.x) * t
    _tiltQuat.y = _tiltQuat.y + (id.y - _tiltQuat.y) * t
    _tiltQuat.z = _tiltQuat.z + (id.z - _tiltQuat.z) * t
    _tiltQuat:normalize()
end

--- Apply yaw delta to desired heading.
--- @param ay number Yaw delta (degrees)
function TRQOrientation.applyYaw(ay)
    _yawDeg = _yawDeg + ay
end

--- Get body-frame pitch from ACTUAL vehicle state (heading-independent).
--- Extracts tilt by removing heading from the up-vector.
--- Convention: returns radians centered at π/2 (90°) when level.
--- @return number Pitch angle in radians
function TRQOrientation.getBodyPitch()
    -- Remove heading: rotate up-vector by negative actual yaw
    local yawRad = math.rad(_actYawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    -- Up-vector in heading-removed frame:
    -- rotZ' = -sinY * upX + cosY * upZ (forward component of tilt)
    local tiltZ = -sinY * _actUpX + cosY * _actUpZ
    -- bodyPitch = acos(magnitude along tilt-Z from vertical)
    -- When level: tiltZ ≈ 0 → acos(0) = π/2. When tilted forward: tiltZ < 0 → acos > π/2.
    return math.acos(clamp(-tiltZ, -1, 1))
end

--- Get body-frame roll from ACTUAL vehicle state (heading-independent).
--- @return number Roll angle in radians
function TRQOrientation.getBodyRoll()
    -- Remove heading: rotate up-vector by negative actual yaw
    local yawRad = math.rad(_actYawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    -- Up-vector in heading-removed frame:
    -- rotX' = cosY * upX + sinY * upZ (right component of tilt)
    local tiltX = cosY * _actUpX + sinY * _actUpZ
    return math.acos(clamp(tiltX, -1, 1))
end

--- Get forward direction from ACTUAL vehicle (not desired).
--- @return number fwdX, number fwdZ Forward direction in PZ world space
function TRQOrientation.getForward()
    return _actFwdX, _actFwdZ
end

--- Get current desired yaw in degrees.
--- @return number Yaw in degrees
function TRQOrientation.getYaw()
    return _yawDeg
end

--- Set yaw to exact value.
--- @param deg number Yaw in degrees
function TRQOrientation.setYaw(deg)
    _yawDeg = deg
end

--- Convert full desired orientation to Euler angles (for debug display).
--- @return number exDeg, number eyDeg, number ezDeg
function TRQOrientation.toEuler()
    return composeOrientation():toEuler()
end

--- Get desired up-vector from TILT ONLY (no heading component).
--- This is heading-independent: the PD tilt error uses this to avoid
--- cross-coupling between yaw lag and tilt correction. Without this,
--- heading lag during yaw creates phantom tilt error proportional to
--- tilt_angle × sin(heading_lag), causing ~500-2000 Nm phantom torque.
--- @return number upX, number upY, number upZ World-frame desired up-vector
function TRQOrientation.getDesiredUpVector()
    if not _tiltQuat then return 0, 1, 0 end
    -- tiltQuat Y-axis = desired up-vector, heading-independent
    local ux, uy, uz = _tiltQuat:vectorY()
    return ux, uy, uz
end

--- Get composed desired orientation quaternion (includes heading, for debug/display).
--- @return Quaternion
function TRQOrientation.getQuaternion()
    return composeOrientation()
end

--- Check if orientation has been initialized.
--- @return boolean
function TRQOrientation.isInitialized()
    return _yawDeg ~= nil
end
