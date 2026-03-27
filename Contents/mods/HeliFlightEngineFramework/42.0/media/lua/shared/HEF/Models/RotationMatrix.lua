--[[
    RotationMatrix — OOP 3x3 rotation matrix with semantic accessors

    PZ convention: Row0=PzX, Row1=PzZ(vertical), Row2=PzY(horizontal).
    Y and Z are SWAPPED compared to standard math convention.

    CONVENTIONS:
    - Convenience methods (toEuler, getPitch, fromPitchYawRoll, etc.) use DEGREES.
      Power-user method (fromEuler) uses RADIANS.
    - vectorY() returns the standard Y axis (PZ's vertical/Z), NOT PZ's horizontal Y.
      Same convention as Quaternion — see Quaternion.lua header for details.
    - PZ-specific accessors (getForwardPzX, getVertical, etc.) use PZ's coordinate
      system directly and are named accordingly.
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

-------------------------------------------------------------------------------------
-- Convenience: individual Euler angles
-------------------------------------------------------------------------------------

--- Get pitch angle (rotation around local X).
--- @return number degrees
function RotationMatrix:getPitch()
    local x, _, _ = self:toEuler()
    return x
end

--- Get yaw angle (rotation around Y).
--- @return number degrees
function RotationMatrix:getYaw()
    local _, y, _ = self:toEuler()
    return y
end

--- Get roll angle (rotation around local Z).
--- @return number degrees
function RotationMatrix:getRoll()
    local _, _, z = self:toEuler()
    return z
end

-------------------------------------------------------------------------------------
-- Convenience: axis vectors (local axes in world space)
-- NOTE: Standard math convention (Y = up). PZ swaps Y/Z.
-- vectorY() = PZ vertical (Z), vectorZ() = PZ horizontal (Y).
-------------------------------------------------------------------------------------

--- Get the local X axis in world space (row 0).
--- @return number x, number y, number z
function RotationMatrix:vectorX()
    return self.m00, self.m01, self.m02
end

--- Get the local Y axis in world space (row 1).
--- In PZ: this is the VERTICAL axis (PZ-Z), not PZ-Y.
--- @return number x, number y, number z
function RotationMatrix:vectorY()
    return self.m10, self.m11, self.m12
end

--- Get the local Z axis in world space (row 2).
--- In PZ: this is the HORIZONTAL axis (PZ-Y), not PZ-Z.
--- @return number x, number y, number z
function RotationMatrix:vectorZ()
    return self.m20, self.m21, self.m22
end

-------------------------------------------------------------------------------------
-- Convenience: constructors
-------------------------------------------------------------------------------------

--- Create from Euler angles (RADIANS). See fromPitchYawRoll for degrees.
--- @param rx number X rotation in radians
--- @param ry number Y rotation in radians
--- @param rz number Z rotation in radians
function RotationMatrix.fromEuler(rx, ry, rz)
    return RotationMatrix.fromQuaternion(Quaternion.fromEuler(rx, ry, rz))
end

--- Create from pitch/yaw/roll (DEGREES). Self-documenting alias.
--- @param pitch number degrees
--- @param yaw number degrees
--- @param roll number degrees
function RotationMatrix.fromPitchYawRoll(pitch, yaw, roll)
    return RotationMatrix.fromEuler(math.rad(pitch), math.rad(yaw), math.rad(roll))
end

-------------------------------------------------------------------------------------
-- Convenience: operations
-------------------------------------------------------------------------------------

--- Multiply two rotation matrices. Returns new RotationMatrix.
--- @param other RotationMatrix
--- @return RotationMatrix
function RotationMatrix:multiply(other)
    local a, b = self, other
    return setmetatable({
        m00 = a.m00*b.m00 + a.m01*b.m10 + a.m02*b.m20,
        m01 = a.m00*b.m01 + a.m01*b.m11 + a.m02*b.m21,
        m02 = a.m00*b.m02 + a.m01*b.m12 + a.m02*b.m22,
        m10 = a.m10*b.m00 + a.m11*b.m10 + a.m12*b.m20,
        m11 = a.m10*b.m01 + a.m11*b.m11 + a.m12*b.m21,
        m12 = a.m10*b.m02 + a.m11*b.m12 + a.m12*b.m22,
        m20 = a.m20*b.m00 + a.m21*b.m10 + a.m22*b.m20,
        m21 = a.m20*b.m01 + a.m21*b.m11 + a.m22*b.m21,
        m22 = a.m20*b.m02 + a.m21*b.m12 + a.m22*b.m22,
    }, RotationMatrix)
end

RotationMatrix.__mul = function(a, b)
    return a:multiply(b)
end

--- Transpose (= inverse for rotation matrices). Returns new RotationMatrix.
--- @return RotationMatrix
function RotationMatrix:transpose()
    return setmetatable({
        m00 = self.m00, m01 = self.m10, m02 = self.m20,
        m10 = self.m01, m11 = self.m11, m12 = self.m21,
        m20 = self.m02, m21 = self.m12, m22 = self.m22,
    }, RotationMatrix)
end

--- Convert to Quaternion. Requires Quaternion to be loaded.
--- @return Quaternion
function RotationMatrix:toQuaternion()
    -- Shepperd's method: numerically stable for all orientations
    local m = self
    local tr = m.m00 + m.m11 + m.m22
    local w, x, y, z
    if tr > 0 then
        local s = math.sqrt(tr + 1) * 2
        w = 0.25 * s
        x = (m.m21 - m.m12) / s
        y = (m.m02 - m.m20) / s
        z = (m.m10 - m.m01) / s
    elseif m.m00 > m.m11 and m.m00 > m.m22 then
        local s = math.sqrt(1 + m.m00 - m.m11 - m.m22) * 2
        w = (m.m21 - m.m12) / s
        x = 0.25 * s
        y = (m.m01 + m.m10) / s
        z = (m.m02 + m.m20) / s
    elseif m.m11 > m.m22 then
        local s = math.sqrt(1 + m.m11 - m.m00 - m.m22) * 2
        w = (m.m02 - m.m20) / s
        x = (m.m01 + m.m10) / s
        y = 0.25 * s
        z = (m.m12 + m.m21) / s
    else
        local s = math.sqrt(1 + m.m22 - m.m00 - m.m11) * 2
        w = (m.m10 - m.m01) / s
        x = (m.m02 + m.m20) / s
        y = (m.m12 + m.m21) / s
        z = 0.25 * s
    end
    return Quaternion.new(w, x, y, z)
end
