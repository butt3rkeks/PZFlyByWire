--[[
    InputProcessor — Key input → rotation deltas

    Reads keyboard state and computes pitch/roll/yaw deltas per frame.
    Handles auto-leveling (proportional to body-frame deviation) and
    wall collision reversal (via framework-provided blocked directions).

    Reads: HeliConfig (tuning), HeliList (per-heli accel/maxSpeed)
    Body angles are passed as parameters (not read from any orientation module).

    Moved from Engines/FBW/FBWInputProcessor.lua to Core/Toolkit/.
    Refactored: body angles (bodyPitch, bodyRoll) are now parameters instead of
    being read from FBWOrientation directly. This makes the module engine-agnostic.
]]

InputProcessor = {}

-------------------------------------------------------------------------------------
-- Compute rotation deltas from key state.
--
-- @param keys table Key state table { up, down, left, right, a, d }
-- @param fpsMultiplier number Frame time normalizer (TARGET_FPS / currentFPS)
-- @param heliType string Helicopter type key for HeliList lookup
-- @param blocked table Framework-provided wall block state { up, down, left, right }
-- @param freeMode boolean Flight assist off (skip auto-level for pitch)
-- @param bodyPitch number Body pitch angle (radians, from orientation module)
-- @param bodyRoll number Body roll angle (radians, from orientation module)
--
-- @return number pitchDelta Pitch delta (degrees)
-- @return number yawDelta Yaw delta (degrees)
-- @return number rollDelta Roll delta (degrees)
-- @return boolean isRotating Whether A/D yaw keys are pressed
-------------------------------------------------------------------------------------
function InputProcessor.computeRotationDeltas(keys, fpsMultiplier, heliType, blocked, freeMode, bodyPitch, bodyRoll)
    local basicAccelerationRate = HeliList[heliType].BasicAccelerationModifier or 0.15
    local maxSpeed = HeliList[heliType].MaxSpeed or 0.3
    if not HeliList[heliType].BasicAccelerationModifier then
        basicAccelerationRate = 0.4
        maxSpeed = 0.15
    end

    local pitchDelta, yawDelta, rollDelta = 0, 0, 0
    local angle_90 = math.rad(90)
    local isRotating = false

    -- Body angles from caller (engine-specific orientation module)
    local angleZ = bodyPitch
    local angleX = bodyRoll

    -- Yaw (A/D keys)
    if keys.a then
        yawDelta = HeliConfig.GetYawRotationSpeed() * fpsMultiplier
        isRotating = true
    end
    if keys.d then
        yawDelta = -HeliConfig.GetYawRotationSpeed() * fpsMultiplier
        isRotating = true
    end

    -- Pitch (UP/DOWN keys)
    if keys.up then
        if not keys.left and not keys.right then
            if angleZ < angle_90 + maxSpeed and not blocked.up then
                pitchDelta = basicAccelerationRate * fpsMultiplier
            else
                pitchDelta = -basicAccelerationRate * fpsMultiplier
            end
        end
    elseif keys.down then
        if not keys.left and not keys.right then
            if angleZ > angle_90 - maxSpeed and not blocked.down then
                pitchDelta = -basicAccelerationRate * fpsMultiplier
            else
                pitchDelta = basicAccelerationRate * fpsMultiplier
            end
        end
    else
        -- Auto-level pitch (skip when FA-off)
        if not freeMode then
            local autoLevelMult = HeliConfig.GetAutoLevelSpeed()
            if angleZ > angle_90 then
                pitchDelta = -basicAccelerationRate * HeliConfig.AUTO_LEVEL_PITCH_FACTOR * (angleZ - angle_90) * fpsMultiplier * autoLevelMult
            elseif angleZ < angle_90 then
                pitchDelta = basicAccelerationRate * HeliConfig.AUTO_LEVEL_PITCH_FACTOR * (angle_90 - angleZ) * fpsMultiplier * autoLevelMult
            end
        end
    end

    -- Roll (LEFT/RIGHT keys)
    if keys.left then
        if not keys.up and not keys.down then
            if angleX < angle_90 + maxSpeed and not blocked.left then
                rollDelta = -basicAccelerationRate * fpsMultiplier
            else
                rollDelta = basicAccelerationRate * fpsMultiplier
            end
        end
    elseif keys.right then
        if not keys.up and not keys.down then
            if angleX > angle_90 - maxSpeed and not blocked.right then
                rollDelta = basicAccelerationRate * fpsMultiplier
            else
                rollDelta = -basicAccelerationRate * fpsMultiplier
            end
        end
    else
        -- Auto-level roll (always active, even in FA-off)
        local autoLevelMult = HeliConfig.GetAutoLevelSpeed()
        if angleX > angle_90 then
            rollDelta = basicAccelerationRate * HeliConfig.AUTO_LEVEL_ROLL_FACTOR * (angleX - angle_90) * fpsMultiplier * autoLevelMult
        elseif angleX < angle_90 then
            rollDelta = -basicAccelerationRate * HeliConfig.AUTO_LEVEL_ROLL_FACTOR * (angle_90 - angleX) * fpsMultiplier * autoLevelMult
        end
    end

    return pitchDelta, yawDelta, rollDelta, isRotating
end

-- Backward compatibility alias
FBWInputProcessor = InputProcessor
