--[[
    CoordUtil — PZ coordinate system helpers

    PZ swaps Y and Z compared to standard math convention:
      PZ X = standard X (horizontal east-west)
      PZ Y = standard Z (horizontal north-south)
      PZ Z = standard Y (vertical up-down)

    Use these helpers to make coordinate intent explicit in engine code,
    rather than relying on comments like "-- PZ Y = world Z".

    Also provides common unit conversions for Bullet physics values.
]]

CoordUtil = {}

-------------------------------------------------------------------------------------
-- Coordinate swaps: convert between PZ and standard math conventions
-------------------------------------------------------------------------------------

--- Convert PZ coordinates (X, Y, Z) to standard math (X, Y, Z).
--- PZ-Y (horizontal) becomes standard-Z, PZ-Z (vertical) becomes standard-Y.
--- @param pzX number PZ X (horizontal)
--- @param pzY number PZ Y (horizontal, north-south)
--- @param pzZ number PZ Z (vertical, up-down)
--- @return number stdX, number stdY, number stdZ
function CoordUtil.pzToStandard(pzX, pzY, pzZ)
    return pzX, pzZ, pzY
end

--- Convert standard math coordinates (X, Y, Z) to PZ (X, Y, Z).
--- Standard-Y (vertical) becomes PZ-Z, standard-Z (horizontal) becomes PZ-Y.
--- @param stdX number Standard X
--- @param stdY number Standard Y (vertical)
--- @param stdZ number Standard Z (horizontal)
--- @return number pzX, number pzY, number pzZ
function CoordUtil.standardToPz(stdX, stdY, stdZ)
    return stdX, stdZ, stdY
end

-------------------------------------------------------------------------------------
-- Unit conversions
-------------------------------------------------------------------------------------

--- Meters per second to kilometers per hour.
--- @param ms number Speed in m/s
--- @return number Speed in km/h
function CoordUtil.msToKmh(ms)
    return ms * 3.6
end

--- Kilometers per hour to meters per second.
--- @param kmh number Speed in km/h
--- @return number Speed in m/s
function CoordUtil.kmhToMs(kmh)
    return kmh / 3.6
end
