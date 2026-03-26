--[[
    Quaternion — OOP quaternion class with metatables

    Reusable by any flight engine. Pre-allocated temp instances for
    zero-alloc hot paths (file-scope locals, never GC'd).

    PZ Kahlua: setmetatable works, __mul metamethod works.
]]

Quaternion = {}
Quaternion.__index = Quaternion

-- Pre-allocated temp instances (file-scope locals, never GC'd)
local _temp1 = { w = 1, x = 0, y = 0, z = 0 }
local _temp2 = { w = 1, x = 0, y = 0, z = 0 }

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

-------------------------------------------------------------------------------------
-- Construction
-------------------------------------------------------------------------------------

function Quaternion.new(w, x, y, z)
    return setmetatable({ w = w or 1, x = x or 0, y = y or 0, z = z or 0 }, Quaternion)
end

function Quaternion.identity()
    return setmetatable({ w = 1, x = 0, y = 0, z = 0 }, Quaternion)
end

function Quaternion.fromAxisAngle(angle, ax, ay, az)
    local s = math.sin(angle * 0.5)
    local c = math.cos(angle * 0.5)
    return setmetatable({ w = c, x = ax * s, y = ay * s, z = az * s }, Quaternion)
end

function Quaternion.fromEuler(rx, ry, rz)
    local c1, c2, c3 = math.cos(rx / 2), math.cos(ry / 2), math.cos(rz / 2)
    local s1, s2, s3 = math.sin(rx / 2), math.sin(ry / 2), math.sin(rz / 2)
    return setmetatable({
        w = c1 * c2 * c3 - s1 * s2 * s3,
        x = s1 * c2 * c3 + c1 * s2 * s3,
        y = c1 * s2 * c3 - s1 * c2 * s3,
        z = c1 * c2 * s3 + s1 * s2 * c3,
    }, Quaternion)
end

-------------------------------------------------------------------------------------
-- Setters
-------------------------------------------------------------------------------------

function Quaternion:set(w, x, y, z)
    self.w = w; self.x = x; self.y = y; self.z = z
    return self
end

function Quaternion:setFrom(other)
    self.w = other.w; self.x = other.x; self.y = other.y; self.z = other.z
    return self
end

function Quaternion:setAxisAngle(angle, ax, ay, az)
    local s = math.sin(angle * 0.5)
    local c = math.cos(angle * 0.5)
    self.w = c; self.x = ax * s; self.y = ay * s; self.z = az * s
    return self
end

-------------------------------------------------------------------------------------
-- Operations (return NEW quaternion)
-------------------------------------------------------------------------------------

function Quaternion:multiply(other)
    local qx, qy, qz, qw = self.x, self.y, self.z, self.w
    local rx, ry, rz, rw = other.x, other.y, other.z, other.w
    return setmetatable({
        w = qw * rw - qx * rx - qy * ry - qz * rz,
        x = qx * rw + qw * rx + qy * rz - qz * ry,
        y = qy * rw + qw * ry + qz * rx - qx * rz,
        z = qz * rw + qw * rz + qx * ry - qy * rx,
    }, Quaternion)
end

Quaternion.__mul = function(q, r)
    return q:multiply(r)
end

function Quaternion:conjugate()
    return setmetatable({ w = self.w, x = -self.x, y = -self.y, z = -self.z }, Quaternion)
end

-------------------------------------------------------------------------------------
-- Operations (write into OUT, zero-alloc)
-------------------------------------------------------------------------------------

function Quaternion:multiplyInto(other, out)
    local qx, qy, qz, qw = self.x, self.y, self.z, self.w
    local rx, ry, rz, rw = other.x, other.y, other.z, other.w
    out.w = qw * rw - qx * rx - qy * ry - qz * rz
    out.x = qx * rw + qw * rx + qy * rz - qz * ry
    out.y = qy * rw + qw * ry + qz * rx - qx * rz
    out.z = qz * rw + qw * rz + qx * ry - qy * rx
    return out
end

function Quaternion:conjugateInto(out)
    out.w = self.w; out.x = -self.x; out.y = -self.y; out.z = -self.z
    return out
end

-------------------------------------------------------------------------------------
-- In-place operations
-------------------------------------------------------------------------------------

function Quaternion:normalize()
    local l = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
    if l == 0 then
        self.x = 0; self.y = 0; self.z = 0; self.w = 1
    else
        l = 1 / l
        self.x = self.x * l; self.y = self.y * l; self.z = self.z * l; self.w = self.w * l
    end
    return self
end

-------------------------------------------------------------------------------------
-- Queries
-------------------------------------------------------------------------------------

function Quaternion:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
end

function Quaternion:unpack()
    return self.x, self.y, self.z, self.w
end

-------------------------------------------------------------------------------------
-- Conversion
-------------------------------------------------------------------------------------

--- Returns 9 rotation matrix components (row-major: R00,R01,R02, R10,R11,R12, R20,R21,R22).
--- PZ convention: Row0=PzX, Row1=PzZ(vertical), Row2=PzY(horizontal). Y/Z SWAPPED.
function Quaternion:toMatrixComponents()
    local q0, q1, q2, q3 = self.w, self.x, self.y, self.z
    return 2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
           2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1),
           2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1
end

--- Convert to RotationMatrix object (requires RotationMatrix to be loaded).
function Quaternion:toMatrix()
    return RotationMatrix.fromQuaternion(self)
end

--- Convert rotation matrix components to Euler angles (degrees).
--- Static utility — takes 9 matrix components, not a quaternion.
function Quaternion.matrixToEuler(m11, m12, m13, m21, m22, m23, m31, m32, m33)
    local y = math.asin(clamp(m13, -1, 1))
    local x, z
    if math.abs(m13) < 0.9999999 then
        x = math.atan2(-m23, m33); z = math.atan2(-m12, m11)
    else
        x = math.atan2(m32, m22); z = 0
    end
    return math.deg(x), math.deg(y), math.deg(z)
end

-------------------------------------------------------------------------------------
-- Pre-allocated temps (for engine-internal zero-alloc paths)
-------------------------------------------------------------------------------------

Quaternion._temp1 = _temp1
Quaternion._temp2 = _temp2
setmetatable(_temp1, Quaternion)
setmetatable(_temp2, Quaternion)
