--[[
    RotationMatrix — OOP 3x3 rotation matrix with semantic accessors

    PZ convention: Row0=PzX, Row1=PzZ(vertical), Row2=PzY(horizontal).
    Y and Z are SWAPPED compared to standard math convention.

    Semantic accessors hide the row mapping so callers never need to
    remember which row is which.
]]

RotationMatrix = {}
RotationMatrix.__index = RotationMatrix

-------------------------------------------------------------------------------------
-- Construction
-------------------------------------------------------------------------------------

--- Create identity matrix.
function RotationMatrix.new()
    return setmetatable({
        m00 = 1, m01 = 0, m02 = 0,
        m10 = 0, m11 = 1, m12 = 0,
        m20 = 0, m21 = 0, m22 = 1,
    }, RotationMatrix)
end

--- Create from a Quaternion (or any table with w,x,y,z fields).
function RotationMatrix.fromQuaternion(q)
    local q0, q1, q2, q3 = q.w, q.x, q.y, q.z
    return setmetatable({
        m00 = 2 * (q0 * q0 + q1 * q1) - 1,
        m01 = 2 * (q1 * q2 - q0 * q3),
        m02 = 2 * (q1 * q3 + q0 * q2),
        m10 = 2 * (q1 * q2 + q0 * q3),
        m11 = 2 * (q0 * q0 + q2 * q2) - 1,
        m12 = 2 * (q2 * q3 - q0 * q1),
        m20 = 2 * (q1 * q3 - q0 * q2),
        m21 = 2 * (q2 * q3 + q0 * q1),
        m22 = 2 * (q0 * q0 + q3 * q3) - 1,
    }, RotationMatrix)
end

-------------------------------------------------------------------------------------
-- Raw access
-------------------------------------------------------------------------------------

--- Get element by row and column (0-indexed).
function RotationMatrix:get(row, col)
    local key = "m" .. row .. col
    return self[key]
end

-------------------------------------------------------------------------------------
-- Semantic accessors (PZ Y/Z swap convention)
-------------------------------------------------------------------------------------

--- Forward direction X component (PZ world X). R02.
function RotationMatrix:getForwardPzX()
    return self.m02
end

--- Forward direction Y component (PZ world Y = horizontal). R22.
function RotationMatrix:getForwardPzY()
    return self.m22
end

--- Vertical component (PZ Z = up). R12.
function RotationMatrix:getVertical()
    return self.m12
end

--- Right-vertical component. R10.
function RotationMatrix:getRightVertical()
    return self.m10
end

-------------------------------------------------------------------------------------
-- Conversion
-------------------------------------------------------------------------------------

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

--- Convert to Euler angles (degrees).
function RotationMatrix:toEuler()
    local y = math.asin(clamp(self.m02, -1, 1))
    local x, z
    if math.abs(self.m02) < 0.9999999 then
        x = math.atan2(-self.m12, self.m22)
        z = math.atan2(-self.m01, self.m00)
    else
        x = math.atan2(self.m21, self.m11)
        z = 0
    end
    return math.deg(x), math.deg(y), math.deg(z)
end
