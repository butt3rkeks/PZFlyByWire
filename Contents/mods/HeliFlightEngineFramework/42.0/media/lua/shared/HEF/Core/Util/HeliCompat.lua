--[[
    HeliCompat — Handler conflict resolution for UH1B Helicopter Physics Fix

    Problem: The WarThunderVehicleLibrary registers an OnTick handler that uses
    debug-only Java reflection (getNumClassFields/getClassField/getClassFieldVal).
    Without -debug, this crashes.

    Defense layers (either one is sufficient):
    1. The library's HeliMove.lua uses :Is() which was renamed to :has() in B42.15.
       Its handler crashes at GetNowMaxZ() → HasFlag() → :Is() before reaching
       setAngles or moveVehicle. Self-destructs harmlessly every frame.
    2. We wrap getNumClassFields in pcall so it returns 0 instead of throwing.
       If a library update fixes :Is() → :has(), the library's moveVehicle still
       becomes a no-op (wTransformFieldNum stays nil because the for loop runs 0
       times, and "if wTransformFieldNum then" skips the body).

    We do NOT override GetHeliType — that would break the library's radial menu
    (Auto-Leveling toggle, NightLight toggle) which also calls GetHeliType.
]]

HeliCompat = {}

-- Wrap getNumClassFields so it never throws in non-debug mode.
-- This global is a PZ built-in (LuaManager @LuaMethod), always available.
if getNumClassFields then
    local _origGetNumClassFields = getNumClassFields
    getNumClassFields = function(obj)
        local ok, result = pcall(_origGetNumClassFields, obj)
        if ok then
            return result
        end
        return 0
    end
end
