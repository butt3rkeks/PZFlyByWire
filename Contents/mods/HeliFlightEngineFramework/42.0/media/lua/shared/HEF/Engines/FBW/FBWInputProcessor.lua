--[[
    FBWInputProcessor — Key input → rotation deltas

    Reads keyboard state and computes pitch/roll/yaw deltas per frame.
    Handles auto-leveling (proportional to body-frame deviation) and
    wall collision reversal (via framework-provided blocked directions).

    Reads: FBWOrientation (body angles), HeliConfig (tuning),
           HeliList (per-heli accel/maxSpeed)
    Does NOT know about: Bullet physics, force math, simulation model, terrain scanning
]]

FBWInputProcessor = {}

-------------------------------------------------------------------------------------
-- Compute rotation deltas from key state.
--
-- @param keys table Key state table { up, down, left, right, a, d }
-- @param fpsMultiplier number Frame time normalizer (TARGET_FPS / currentFPS)
-- @param heliType string Helicopter type key for HeliList lookup
-- @param blocked table Framework-provided wall block state { up, down, left, right }
-- @param freeMode boolean Flight assist off (skip auto-level for pitch)
--
-- @return number ax Pitch delta (degrees)
-- @return number ay Yaw delta (degrees)
-- @return number az Roll delta (degrees)
-- @return boolean isRotating Whether A/D yaw keys are pressed
-------------------------------------------------------------------------------------
function FBWInputProcessor.computeRotationDeltas(keys, fpsMultiplier, heliType, blocked, freeMode)
    local basicAccel = HeliList[heliType].BasicAccelerationModifier or 0.15
    local maxSpeed = HeliList[heliType].MaxSpeed or 0.3
    if not HeliList[heliType].BasicAccelerationModifier then
        basicAccel = 0.4
        maxSpeed = 0.15
    end

    local ax, ay, az = 0, 0, 0
    local angle_90 = math.rad(90)
    local isRotating = false

    -- Read body angles from FBWOrientation (heading-independent, near identity)
    local angleZ = FBWOrientation.getBodyPitch()
    local angleX = FBWOrientation.getBodyRoll()

    -- Yaw (A/D keys)
    if keys.a then
        ay = HeliConfig.YAW_SPEED * fpsMultiplier
        isRotating = true
    end
    if keys.d then
        ay = -HeliConfig.YAW_SPEED * fpsMultiplier
        isRotating = true
    end

    -- Pitch (UP/DOWN keys)
    if keys.up then
        if not keys.left and not keys.right then
            if angleZ < angle_90 + maxSpeed and not blocked.up then
                ax = basicAccel * fpsMultiplier
            else
                ax = -basicAccel * fpsMultiplier
            end
        end
    elseif keys.down then
        if not keys.left and not keys.right then
            if angleZ > angle_90 - maxSpeed and not blocked.down then
                ax = -basicAccel * fpsMultiplier
            else
                ax = basicAccel * fpsMultiplier
            end
        end
    else
        -- Auto-level pitch (skip when FA-off)
        if not freeMode then
            local autoLevelMult = HeliConfig.get("autolevel")
            if angleZ > angle_90 then
                ax = -basicAccel * HeliConfig.AUTO_LEVEL_PITCH_FACTOR * (angleZ - angle_90) * fpsMultiplier * autoLevelMult
            elseif angleZ < angle_90 then
                ax = basicAccel * HeliConfig.AUTO_LEVEL_PITCH_FACTOR * (angle_90 - angleZ) * fpsMultiplier * autoLevelMult
            end
        end
    end

    -- Roll (LEFT/RIGHT keys)
    if keys.left then
        if not keys.up and not keys.down then
            if angleX < angle_90 + maxSpeed and not blocked.left then
                az = -basicAccel * fpsMultiplier
            else
                az = basicAccel * fpsMultiplier
            end
        end
    elseif keys.right then
        if not keys.up and not keys.down then
            if angleX > angle_90 - maxSpeed and not blocked.right then
                az = basicAccel * fpsMultiplier
            else
                az = -basicAccel * fpsMultiplier
            end
        end
    else
        -- Auto-level roll (always active, even in FA-off)
        local autoLevelMult = HeliConfig.get("autolevel")
        if angleX > angle_90 then
            az = basicAccel * HeliConfig.AUTO_LEVEL_ROLL_FACTOR * (angleX - angle_90) * fpsMultiplier * autoLevelMult
        elseif angleX < angle_90 then
            az = -basicAccel * HeliConfig.AUTO_LEVEL_ROLL_FACTOR * (angle_90 - angleX) * fpsMultiplier * autoLevelMult
        end
    end

    return ax, ay, az, isRotating
end
