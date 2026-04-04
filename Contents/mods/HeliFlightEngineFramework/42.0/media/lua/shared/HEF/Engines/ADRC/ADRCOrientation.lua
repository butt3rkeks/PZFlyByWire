--[[
    ADRCOrientation -- Desired orientation tracker for the ADRC engine

    Manages the desired orientation as separated yaw (scalar) + pitch/roll (scalars).
    Tilt decays toward level when no directional input is held.
    Body angles for input clamping come from ACTUAL vehicle state (Bullet),
    not from the desired orientation (torque always lags desired).

    IMPORTANT: Tilt is stored as scalar pitch/roll angles, NOT as an accumulated
    quaternion. The full desired quaternion is reconstructed each frame from
    yaw + pitch + roll. This eliminates path-dependent spiral drift that occurred
    when accumulating quaternion increments at changing headings (simultaneous
    pitch + yaw created phantom roll from different-axis accumulation).

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
-- Desired state: yaw + pitch + roll as scalars (degrees)
-------------------------------------------------------------------------------------
local _yawDeg = nil
local _pitchDeg = 0    -- desired pitch angle (degrees, positive = forward)
local _rollDeg = 0     -- desired roll angle (degrees, positive = right)
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
-- Compose full desired orientation from yaw + pitch + roll scalars.
-- Reconstructed fresh each frame — no path-dependent accumulation.
-- Order: yaw (world Y) * pitch (heading-relative right axis) * roll (heading-relative forward axis)
-------------------------------------------------------------------------------------
local function composeOrientation()
    local yawQ = Quaternion.fromAxisAngle(math.rad(_yawDeg), 0, 1, 0)
    -- Pitch around body-right at current heading
    local yawRad = math.rad(_yawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    local pitchQ = Quaternion.fromAxisAngle(math.rad(_pitchDeg), cosY, 0, -sinY)
    -- Roll around body-forward at current heading
    local rollQ = Quaternion.fromAxisAngle(math.rad(_rollDeg), sinY, 0, cosY)
    return pitchQ * rollQ * yawQ
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
    _pitchDeg = 0
    _rollDeg = 0
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
    _pitchDeg = 0
    _rollDeg = 0
    _initialized = false
    _actUpX = 0; _actUpY = 1; _actUpZ = 0
    _actFwdX = 0; _actFwdY = 0; _actFwdZ = 1
    _actYawDeg = 0
end

--- Apply tilt deltas to desired pitch/roll angles.
--- Heading-independent: pitch is always forward/back, roll is always left/right
--- regardless of heading. No quaternion accumulation — scalars are path-independent.
--- @param pitchDelta number Pitch delta (degrees, positive = tilt forward)
--- @param rollDelta number Roll delta (degrees, positive = tilt right)
function ADRCOrientation.applyTilt(pitchDelta, rollDelta)
    _pitchDeg = _pitchDeg + pitchDelta
    _rollDeg = _rollDeg + rollDelta
end

--- Decay desired tilt toward level when no directional input.
--- @param rate number Decay alpha (0-1 per frame)
function ADRCOrientation.decayTiltToLevel(rate)
    local t = math.min(rate, 1.0)
    _pitchDeg = _pitchDeg * (1 - t)
    _rollDeg = _rollDeg * (1 - t)
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
    local tiltZ = -sinY * _actUpX + cosY * _actUpZ
    return math.acos(clamp(-tiltZ, -1, 1))
end

--- Get body-frame roll from ACTUAL vehicle state (heading-independent).
--- @return number Roll angle in radians
function ADRCOrientation.getBodyRoll()
    local yawRad = math.rad(_actYawDeg)
    local cosY = math.cos(yawRad)
    local sinY = math.sin(yawRad)
    local tiltX = cosY * _actUpX + sinY * _actUpZ
    return math.acos(clamp(tiltX, -1, 1))
end

--- Get desired up-vector from TILT ONLY (no heading component).
--- @return number upX, number upY, number upZ World-frame desired up-vector
function ADRCOrientation.getDesiredUpVector()
    local pitchQ = Quaternion.fromAxisAngle(math.rad(_pitchDeg), 1, 0, 0)
    local rollQ = Quaternion.fromAxisAngle(math.rad(_rollDeg), 0, 0, 1)
    local tiltQ = pitchQ * rollQ
    return tiltQ:vectorY()
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
--- Reconstructed fresh — no accumulated drift.
--- @return Quaternion
function ADRCOrientation.getQuaternion()
    return composeOrientation()
end

--- Convert full desired orientation to Euler angles (for debug display).
--- @return number exDeg, number eyDeg, number ezDeg
function ADRCOrientation.toEuler()
    return composeOrientation():toEuler()
end
