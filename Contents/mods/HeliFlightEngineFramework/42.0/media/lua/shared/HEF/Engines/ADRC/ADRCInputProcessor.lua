--[[
    ADRCInputProcessor — Decoupled input handling for the ADRC engine

    Owns all raw key reading and produces processed outputs:
      - Smoothed tilt rates (pitch/roll deltas per frame)
      - Smoothed yaw rate (yaw delta per frame)
      - Ramped vertical target (asymmetric: fast ramp to input, slow coast to hover)

    The engine reads only processed values, never raw keys.
    First-order rate smoothing prevents ADRC torque saturation on key press/release.
    Asymmetric vertical ramp reduces transient coupling with tilt during W/S transitions.
]]

ADRCInputProcessor = {}

local exp = math.exp
local abs = math.abs
local rad = math.rad

-------------------------------------------------------------------------------------
-- State (module-level, persistent across frames)
-------------------------------------------------------------------------------------
local _smoothedYawRate = 0
local _smoothedPitchRate = 0
local _smoothedRollRate = 0
local _rampedTargetVelY = 0

-------------------------------------------------------------------------------------
-- Reusable output table (zero-alloc hot path)
-------------------------------------------------------------------------------------
local _out = {
    pitchDelta = 0, rollDelta = 0, yawDelta = 0,
    isRotating = false, hasTiltInput = false,
    targetVelY = 0, gravComp = false, vBraking = false, engineDead = false,
    rawTargetVelY = 0,
}

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Process all input for one frame. Reads raw keys, produces smoothed outputs.
--- @param keys table Raw key state {w, s, a, d, up, down, left, right}
--- @param ctx table HEFContext
--- @param freeMode boolean Flight assist off
--- @param bodyPitch number Current body pitch angle (radians, from ADRCOrientation)
--- @param bodyRoll number Current body roll angle (radians, from ADRCOrientation)
--- @return table Processed input {pitchDelta, rollDelta, yawDelta, isRotating,
---   hasTiltInput, targetVelY, gravComp, vBraking, engineDead, rawTargetVelY}
function ADRCInputProcessor.update(keys, ctx, freeMode, bodyPitch, bodyRoll)
    local heliType = ctx.heliType
    local blocked = ctx.blocked
    local fpsMultiplier = ctx.fpsMultiplier

    -- HeliList vehicle parameters
    local basicAccelRate = HeliList[heliType].BasicAccelerationModifier or 0.15
    local maxSpeed = HeliList[heliType].MaxSpeed or 0.3
    if not HeliList[heliType].BasicAccelerationModifier then
        basicAccelRate = 0.4; maxSpeed = 0.15
    end
    local angle_90 = rad(90)

    -- === YAW (A/D) ===
    local rawYawRate = 0
    local isRotating = false
    if keys.a then rawYawRate = HeliConfig.GetYawRotationSpeed(); isRotating = true end
    if keys.d then rawYawRate = -HeliConfig.GetYawRotationSpeed(); isRotating = true end

    -- === PITCH (UP/DOWN) ===
    local rawPitchRate = 0
    if keys.up and not keys.left and not keys.right then
        if bodyPitch < angle_90 + maxSpeed and not blocked.up then
            rawPitchRate = basicAccelRate
        end
    elseif keys.down and not keys.left and not keys.right then
        if bodyPitch > angle_90 - maxSpeed and not blocked.down then
            rawPitchRate = -basicAccelRate
        end
    end

    -- === ROLL (LEFT/RIGHT) ===
    local rawRollRate = 0
    if keys.left and not keys.up and not keys.down then
        if bodyRoll < angle_90 + maxSpeed and not blocked.left then
            rawRollRate = -basicAccelRate
        end
    elseif keys.right and not keys.up and not keys.down then
        if bodyRoll > angle_90 - maxSpeed and not blocked.right then
            rawRollRate = basicAccelRate
        end
    end

    -- === FIRST-ORDER RATE SMOOTHING ===
    local tau = HeliConfig.GetAdrcInputSmoothingTau()
    local dt_input = 1.0 / ctx.fps
    if tau > 0.001 then
        local alpha = 1.0 - exp(-dt_input / tau)
        _smoothedYawRate   = _smoothedYawRate   + (rawYawRate   - _smoothedYawRate)   * alpha
        _smoothedPitchRate = _smoothedPitchRate + (rawPitchRate - _smoothedPitchRate) * alpha
        _smoothedRollRate  = _smoothedRollRate  + (rawRollRate  - _smoothedRollRate)  * alpha
    else
        _smoothedYawRate   = rawYawRate
        _smoothedPitchRate = rawPitchRate
        _smoothedRollRate  = rawRollRate
    end

    -- === FPS-MULTIPLIER SCALING ===
    local pitchDelta = _smoothedPitchRate * fpsMultiplier
    local rollDelta  = _smoothedRollRate  * fpsMultiplier
    local yawDelta   = _smoothedYawRate   * fpsMultiplier

    local hasTiltInput = keys.up or keys.down or keys.left or keys.right

    -- === VERTICAL TARGET ===
    local rawTargetVelY, gravComp, vBraking, engineDead = FlightModel.computeVerticalTarget(ctx, freeMode)

    -- Landing zone taper
    local currentAltitude = ctx.currentAltitude
    local groundLevelZ = ctx.groundLevelZ
    if rawTargetVelY < 0 and currentAltitude < groundLevelZ + HeliConfig.GetAdrcLandingZoneHeight() then
        local landingFactor = math.max((currentAltitude - groundLevelZ) / HeliConfig.GetAdrcLandingZoneHeight(), 0)
        landingFactor = math.max(landingFactor, HeliConfig.GetAdrcLandingMinSpeedFactor())
        rawTargetVelY = rawTargetVelY * landingFactor
    end

    -- Asymmetric ramp: fast toward input (responsive), slow toward zero (smooth coast)
    local dt_ramp = 1.0 / ctx.fps
    local rampRate
    if abs(rawTargetVelY) > abs(_rampedTargetVelY) then
        rampRate = 4.0   -- key pressed: 0.25s tau
    else
        rampRate = 1.5   -- key released: 0.67s tau (coast to hover)
    end
    _rampedTargetVelY = _rampedTargetVelY + (rawTargetVelY - _rampedTargetVelY) * (1.0 - exp(-rampRate * dt_ramp))

    -- === POPULATE OUTPUT ===
    _out.pitchDelta = pitchDelta
    _out.rollDelta = rollDelta
    _out.yawDelta = yawDelta
    _out.isRotating = isRotating
    _out.hasTiltInput = hasTiltInput
    _out.targetVelY = _rampedTargetVelY
    _out.gravComp = gravComp
    _out.vBraking = vBraking
    _out.engineDead = engineDead
    _out.rawTargetVelY = rawTargetVelY
    return _out
end

--- Reset all smoothing state. Call on flight init / liftoff.
function ADRCInputProcessor.reset()
    _smoothedYawRate = 0
    _smoothedPitchRate = 0
    _smoothedRollRate = 0
    _rampedTargetVelY = 0
end
