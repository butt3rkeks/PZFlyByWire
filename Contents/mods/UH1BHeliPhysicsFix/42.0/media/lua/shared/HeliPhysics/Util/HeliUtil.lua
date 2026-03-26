--[[
    HeliUtil — Shared utility functions for helicopter physics mod.

    Loaded early (shared/ loads before client/) and available to all modules.
]]

HeliUtil = {}

--- Coerce a Java Float to a Lua number.
--- Kahlua's tonumber() fails on Java objects, and arithmetic (+0) may return
--- another Java Float. String roundtrip is the only reliable conversion:
--- tostring(JavaFloat) → "123.456" → tonumber("123.456") → Lua number.
--- @param v any Value to coerce (typically from vehicle:getX(), getAngleY(), etc.)
--- @return number Lua number, or 0 if conversion fails
function HeliUtil.toLuaNum(v)
    if v == nil then return 0 end
    local n = tonumber(tostring(v))
    return n or 0
end

--- Normalize an angle to [-180, 180) range.
--- Handles any input value including multi-rotation accumulation (e.g., 720°).
--- @param angle number Angle in degrees
--- @return number Normalized angle in [-180, 180)
function HeliUtil.normalizeAngle(angle)
    angle = angle % 360  -- reduce to [0, 360) — NOTE: Kahlua uses C-style modulo
    if angle < 0 then angle = angle + 360 end  -- ensure positive for C-style modulo
    if angle >= 180 then angle = angle - 360 end  -- shift to [-180, 180)
    return angle
end
