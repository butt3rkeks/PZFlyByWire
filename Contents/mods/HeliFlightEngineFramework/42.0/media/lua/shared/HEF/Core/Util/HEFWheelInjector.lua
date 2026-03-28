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
      - Centered at X=0 → eliminates asymmetric btRaycastVehicle torque that
        caused tilt drift with spread wheels during descent
      - Vehicle-level suspension must be zeroed in .txt → no ground interaction forces
      - Sole purpose: getWheelCount() > 0 in updateVelocityMultiplier()

    No registration required. Any helicopter registered in HeliList is covered
    automatically, including helicopters added by third-party mods.
]]

HEFWheelInjector = {}

local PHANTOM_WHEEL = [[{
    wheel PhantomCenter
    {
        front = TRUE,
        offset = 0.000000 0.000000 0.000000,
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
}]]

local function injectPhantomWheelIntoWarThunderHelis()
    if not HeliList then return end
    local sm = getScriptManager()
    if not sm then return end

    for heliName, _ in pairs(HeliList) do
        local script = sm:getVehicle("Base." .. heliName)
        if script then
            if script:getWheelCount() == 0 then
                local ok, err = pcall(function()
                    script:Load(script:getFullName(), PHANTOM_WHEEL)
                end)
                if ok then
                    print("[HEF] WheelInjector: injected phantom wheel into " .. script:getFullName())
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
