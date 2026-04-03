--[[
    ADRCOrientation -- Desired orientation tracker for the ADRC engine

    Manages the desired orientation as separated yaw (scalar) + tilt (quaternion).
    Tilt decays toward level when no directional input is held.
    Body angles for input clamping come from ACTUAL vehicle state (Bullet),
    not from the desired orientation (torque always lags desired).

    Uses Models/Quaternion for quaternion math.
    Depends only on HeliConfig (via ADRCHeliConfig getters).
]]

ADRCOrientation = {}

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

-------------------------------------------------------------------------------------
-- Desired state: separated yaw (scalar) + tilt (quaternion)
-------------------------------------------------------------------------------------
local _yawDeg = nil
local _tiltQuat = nil
local _initialized = false

-------------------------------------------------------------------------------------
-- Actual vehicle state (updated from Bullet each frame via updateActualState)
-------------------------------------------------------------------------------------
local _actUpX = 0
local _actUpY = 1
local _actUpZ = 0
local _actFwdX = 0
local _actFwdY = 0
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
function ADRCOrientation.initFromVehicle(angleX, angleY, angleZ)
    local fullQ = Quaternion.fromEuler(
        math.rad(angleX), math.rad(angleY), math.rad(angleZ))
    local fwdX = 2 * (fullQ.x * fullQ.z + fullQ.w * fullQ.y)
    local fwdZ = 1 - 2 * (fullQ.x * fullQ.x + fullQ.y * fullQ.y)
    _yawDeg = math.deg(math.atan2(fwdX, fwdZ))
    _tiltQuat = Quaternion.identity()
    _initialized = true
end

--- Update actual vehicle state from Bullet readings.
--- Must be called each frame BEFORE input processing or body angle queries.
--- @param upX number Actual up-vector X (world frame)
--- @param upY number Actual up-vector Y
--- @param upZ number Actual up-vector Z
--- @param fwdX number Actual forward X
--- @param fwdY number Actual forward Y
--- @param fwdZ number Actual forward Z
--- @param yawDeg number Actual yaw (degrees)
function ADRCOrientation.updateActualState(upX, upY, upZ, fwdX, fwdY, fwdZ, yawDeg)
    _actUpX = upX
    _actUpY = upY
    _actUpZ = upZ
    _actFwdX = fwdX
    _actFwdY = fwdY
    _actFwdZ = fwdZ
    _actYawDeg = yawDeg
end

--- @return boolean
function ADRCOrientation.isInitialized()
    return _initialized
end

--- Reset all orientation state.
function ADRCOrientation.reset()
    _yawDeg = nil
    _tiltQuat = nil
    _initialized = false
    _actUpX = 0; _actUpY = 1; _actUpZ = 0
    _actFwdX = 0; _actFwdY = 0; _actFwdZ = 1
    _actYawDeg = 0
end

--- Apply body-frame tilt corrections to the desired orientation.
--- Tilt axes are rotated by the current heading so pitch is always forward/back
--- and roll is always left/right regardless of which direction the helicopter faces.
--- Without this, pitch at heading -86° would produce roll (world X ≈ body forward).
--- @param pitchDelta number Pitch delta (degrees, positive = tilt forward)
--- @param rollDelta number Roll delta (degrees, positive = tilt right)
function ADRCOrientation.applyTilt(pitchDelta, rollDelta)
    if pitchDelta ~= 0 or rollDelta ~= 0 then
        -- Heading-rotated body axes (horizontal plane):
        --   body right = (cos(yaw), 0, -sin(yaw))
        --   body forward = (sin(yaw), 0, cos(yaw))
        local yawRad = math.rad(_yawDeg)
        local cosY = math.cos(yawRad)
        local sinY = math.sin(yawRad)

        -- Pitch: rotate around body-right axis (perpendicular to heading)
        if pitchDelta ~= 0 then
            local nqp = Quaternion.fromAxisAngle(math.rad(pitchDelta), cosY, 0, -sinY)
            _tiltQuat = _tiltQuat * nqp
        end
        -- Roll: rotate around body-forward axis (along heading)
        if rollDelta ~= 0 then
            local nqr = Quaternion.fromAxisAngle(math.rad(rollDelta), sinY, 0, cosY)
            _tiltQuat = _tiltQuat * nqr
        end
        _tiltQuat:normalize()
    end
end

--- Decay desired tilt toward level when no directional input.
--- Uses LERP toward identity quaternion at the given rate.
--- @param rate number Decay alpha (0-1 per frame)
function ADRCOrientation.decayTiltToLevel(rate)
    if not _tiltQuat then return end
    local t = math.min(rate, 1.0)
    local id = Quaternion.identity()
    _tiltQuat.w = _tiltQuat.w + (id.w - _tiltQuat.w) * t
    _tiltQuat.x = _tiltQuat.x + (id.x - _tiltQuat.x) * t
    _tiltQuat.y = _tiltQuat.y + (id.y - _tiltQuat.y) * t
    _tiltQuat.z = _tiltQuat.z + (id.z - _tiltQuat.z) * t
    _tiltQuat:normalize()
end

--- Apply yaw delta to desired heading.
--- @param yawDelta number Yaw delta (degrees)
function ADRCOrientation.applyYaw(yawDelta)
    _yawDeg = _yawDeg + yawDelta
end

--- Get body-frame pitch from ACTUAL vehicle state (heading-independent).
--- Convention: returns radians centered at pi/2 when level.
--- @return number Pitch angle in radians
function ADRCOrientation.getBodyPitch()
    local yawRad = math.rad(_actYawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    -- Forward component of tilt after removing heading
    local tiltZ = -sinY * _actUpX + cosY * _actUpZ
    return math.acos(clamp(-tiltZ, -1, 1))
end

--- Get body-frame roll from ACTUAL vehicle state (heading-independent).
--- @return number Roll angle in radians
function ADRCOrientation.getBodyRoll()
    local yawRad = math.rad(_actYawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    -- Right component of tilt after removing heading
    local tiltX = cosY * _actUpX + sinY * _actUpZ
    return math.acos(clamp(tiltX, -1, 1))
end

--- Get desired up-vector from TILT ONLY (no heading component).
--- Heading-independent: avoids cross-coupling between yaw lag and tilt correction.
--- @return number upX, number upY, number upZ World-frame desired up-vector
function ADRCOrientation.getDesiredUpVector()
    if not _tiltQuat then return 0, 1, 0 end
    return _tiltQuat:vectorY()
end

--- Get current desired yaw in degrees.
--- @return number Yaw in degrees
function ADRCOrientation.getYaw()
    return _yawDeg
end

--- Set yaw to exact value (used by yaw controller lock).
--- @param deg number Yaw in degrees
function ADRCOrientation.setYaw(deg)
    _yawDeg = deg
end

--- Get forward direction from ACTUAL vehicle (not desired).
--- @return number fwdX, number fwdZ Forward direction components
function ADRCOrientation.getForward()
    return _actFwdX, _actFwdZ
end

--- Get actual yaw from vehicle state.
--- @return number Actual yaw in degrees
function ADRCOrientation.getActualYaw()
    return _actYawDeg
end

--- Get actual up-vector from vehicle state.
--- @return number upX, number upY, number upZ
function ADRCOrientation.getActualUp()
    return _actUpX, _actUpY, _actUpZ
end

--- Get composed desired orientation quaternion (includes heading).
--- @return Quaternion
function ADRCOrientation.getQuaternion()
    return composeOrientation()
end

--- Convert full desired orientation to Euler angles (for debug display).
--- @return number exDeg, number eyDeg, number ezDeg
function ADRCOrientation.toEuler()
    return composeOrientation():toEuler()
end
