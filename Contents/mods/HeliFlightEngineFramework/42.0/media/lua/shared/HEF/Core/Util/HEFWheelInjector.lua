--[[
    HEFWheelInjector — Runtime phantom-wheel injection for the velocity-dampener bypass

    Problem: PZ's updateVelocityMultiplier() (BaseVehicle.java) applies a 0.1x velocity
    multiplier every frame after each physics step for vehicles with 0 wheels. This
    zeros out any velocity the flight engine builds up, making flight impossible.

    Fix: on OnGameStart, iterate HeliList (the WarThunder Vehicle Library's global
    helicopter registry) and inject a phantom wheel into every helicopter VehicleScript
    that still has 0 wheels. VehicleScript is a shared, parse-once singleton — the
    injection is visible to all future spawns of that vehicle type.

    Timing: OnGameStart fires AFTER chunk loading (save-file vehicles already have
    Bullet physics initialized), but BEFORE any new spawns. Injected wheels are seen
    by all helicopters spawned after framework boot.

    Compatibility: if a helicopter .txt already defines a wheel, getWheelCount()
    returns > 0 and the injector skips it. Safe to have both.

    Phantom wheel design (intentional):
      - No model field → invisible, never rendered
      - Positioned at centerOfMassOffset → eliminates asymmetric moment arm that
        caused directional tilt bias during ascent/descent. With offset (0,0,0) the
        wheel was 7m forward of CoM on UH-1B, giving a pitch-axis moment arm.
      - Vehicle-level suspension must be zeroed in .txt → no ground interaction forces
      - Sole purpose: getWheelCount() > 0 in updateVelocityMultiplier()

    No registration required. Any helicopter registered in HeliList is covered
    automatically, including helicopters added by third-party mods.
]]

HEFWheelInjector = {}

local toLuaNum = HeliUtil and HeliUtil.toLuaNum or tonumber

--- Read the centerOfMassOffset from a VehicleScript.
--- Returns x, y, z in PZ script coordinates, or 0,0,0 if unavailable.
local function readCoMOffset(script)
    local ok, com = pcall(function() return script:getCenterOfMassOffset() end)
    if not ok or not com then return 0, 0, 0 end

    -- Try :x()/:y()/:z() first, then :getX()/:getY()/:getZ(), then string parse
    local function readComp(vec, comp)
        local s, v = pcall(function() return toLuaNum(vec[comp](vec)) end)
        if s and v then return v end
        s, v = pcall(function()
            local getter = "get" .. comp:upper()
            return toLuaNum(vec[getter](vec))
        end)
        if s and v then return v end
        return nil
    end

    local cx = readComp(com, "x")
    local cy = readComp(com, "y")
    local cz = readComp(com, "z")
    if cx and cy and cz then return cx, cy, cz end

    -- Fallback: parse tostring
    local s = tostring(com)
    if s then
        local a, b, c = s:match("([%d%.%-]+)[,%s]+([%d%.%-]+)[,%s]+([%d%.%-]+)")
        if a then return tonumber(a) or 0, tonumber(b) or 0, tonumber(c) or 0 end
    end
    return 0, 0, 0
end

--- Build phantom wheel script text with given offset (PZ script coordinates).
local function buildPhantomWheelScript(offX, offY, offZ)
    return string.format([[{
    wheel PhantomCenter
    {
        front = TRUE,
        offset = %.6f %.6f %.6f,
        radius = 0.300000,
        width = 0.200000,
    },
    suspensionStiffness = 0,
    suspensionCompression = 0,
    suspensionDamping = 0,
    maxSuspensionTravelCm = 0,
    suspensionRestLength = 0,
    rollInfluence = 0,
    wheelFriction = 0,
    stoppingMovementForce = 0,
}]], offX, offY, offZ)
end

-- Phantom wheel values: all zeros. Verified safe — Bullet's btRaycastVehicle uses
-- suspensionStiffness, suspensionDamping, wheelFriction as MULTIPLIERS only (never
-- divisors). Zero produces zero force, no NaN risk. Source: btRaycastVehicle.cpp
-- updateSuspension (stiffness × length_diff) and updateFriction (frictionSlip × suspForce).

local function injectPhantomWheelIntoWarThunderHelis()
    if not HeliList then return end
    local sm = getScriptManager()
    if not sm then return end

    for heliName, _ in pairs(HeliList) do
        local script = sm:getVehicle("Base." .. heliName)
        if script then
            if script:getWheelCount() == 0 then
                -- Read CoM offset so phantom wheel sits at CoM (zero moment arm)
                local comX, comY, comZ = readCoMOffset(script)
                local wheelScript = buildPhantomWheelScript(comX, comY, comZ)

                local ok, err = pcall(function()
                    script:Load(script:getFullName(), wheelScript)
                end)
                if ok then
                    if comX ~= 0 or comY ~= 0 or comZ ~= 0 then
                        print("[HEF] WheelInjector: injected phantom wheel into " .. script:getFullName()
                            .. " at CoM (" .. string.format("%.3f, %.3f, %.3f", comX, comY, comZ) .. ")")
                    else
                        print("[HEF] WheelInjector: injected phantom wheel into " .. script:getFullName()
                            .. " at origin (no CoM offset)")
                    end
                else
                    print("[HEF] WheelInjector: ERROR injecting into " .. heliName .. ": " .. tostring(err))
                end
            end
        else
            print("[HEF] WheelInjector: script not found for HeliList entry '" .. heliName .. "' (tried Base." .. heliName .. ")")
        end
    end
end

Events.OnGameStart.Add(injectPhantomWheelIntoWarThunderHelis)
