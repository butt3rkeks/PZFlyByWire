--[[
    HeliVelocityAdapter — Bullet velocity reading + smoothing

    Reads velocity from Bullet's getLinearVelocity with stale-read detection.
    Bullet returns stale horizontal values on ~2/3 of frames (~3.5 m/s when
    actual is 30+ m/s). Position-based velocity is used as ground truth when
    the Bullet read disagrees by > 3x.

    Also provides position-delta speed (ground truth, immune to stale reads).
    Unifies position-delta tracking into a single adapter.
]]

HeliVelocityAdapter = {}

-------------------------------------------------------------------------------------
-- Internal state
-------------------------------------------------------------------------------------
local _tempVelVec = nil

-- Velocity smoothing
local _prevPosX = nil
local _prevPosZ = nil
local _prevFPS = 60
local _smoothVelX = 0
local _smoothVelZ = 0

-- Position-delta speed (unified tracker)
local _posDeltaSpeed = 0

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Get smoothed horizontal velocity and raw vertical velocity.
--- Also updates position-delta speed as a side effect.
--- @param vehicle BaseVehicle
--- @return number smoothVelX, number rawVelY, number smoothVelZ
function HeliVelocityAdapter.getVelocity(vehicle)
    if not _tempVelVec then _tempVelVec = Vector3f.new() end
    local vel = vehicle:getLinearVelocity(_tempVelVec)
    local rawVx = vel:x()
    local rawVy = vel:y()
    local rawVz = vel:z()

    local toLuaNum = HeliUtil.toLuaNum

    -- Position-based horizontal velocity (ground truth)
    local posX = toLuaNum(vehicle:getX())
    local posZ = toLuaNum(vehicle:getY())  -- PZ Y = world Z
    local fps = math.max(getAverageFPS(), HeliConfig.MIN_FPS)

    if _prevPosX then
        local dx = posX - _prevPosX
        local dz = posZ - _prevPosZ
        local posDeltaVelX = dx * _prevFPS
        local posDeltaVelZ = dz * _prevFPS

        -- Update position-delta speed (ground truth, used by snap guard)
        _posDeltaSpeed = math.sqrt(dx * dx + dz * dz) * fps

        local bulletSpeedSq = toLuaNum(rawVx) * toLuaNum(rawVx) + toLuaNum(rawVz) * toLuaNum(rawVz)
        local posSpeedSq = posDeltaVelX * posDeltaVelX + posDeltaVelZ * posDeltaVelZ

        -- If position says we're moving but Bullet says we're nearly stopped,
        -- the Bullet read is stale. Use position-based velocity.
        if posSpeedSq > 25 and bulletSpeedSq < posSpeedSq * 0.1 then
            _smoothVelX = posDeltaVelX
            _smoothVelZ = posDeltaVelZ
        else
            _smoothVelX = toLuaNum(rawVx)
            _smoothVelZ = toLuaNum(rawVz)
        end
    else
        _smoothVelX = toLuaNum(rawVx)
        _smoothVelZ = toLuaNum(rawVz)
        _posDeltaSpeed = 0
    end

    _prevPosX = posX
    _prevPosZ = posZ
    _prevFPS = fps

    return _smoothVelX, rawVy, _smoothVelZ
end

--- Get position-delta speed (m/s). Ground truth, immune to stale Bullet reads.
--- Updated each frame by getVelocity().
--- @return number Speed in m/s
function HeliVelocityAdapter.getPositionDeltaSpeed()
    return _posDeltaSpeed
end

--- Reset smoothing state (called on flight init).
function HeliVelocityAdapter.resetSmoothing()
    _prevPosX = nil
    _prevPosZ = nil
    _smoothVelX = 0
    _smoothVelZ = 0
    _posDeltaSpeed = 0
end
