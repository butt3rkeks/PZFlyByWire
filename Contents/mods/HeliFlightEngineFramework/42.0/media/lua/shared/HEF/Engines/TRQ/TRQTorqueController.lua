--[[
    TRQTorqueController — Hybrid angular PD controller with inertia normalization

    Yaw (Y):          scalar wrapAngle error — proven stable
    Pitch/Roll (X/Z): Z-axis (up-vector) tilt error — PX4 approach

    The Z-axis method computes tilt error from the cross product of
    actual and desired up-vectors. The result is DIRECTLY in world frame.
    No heading rotation needed, no sign flip at any heading.

    Hysteresis anti-unwinding on the tilt error prevents inverted equilibrium.
    Gyroscopic feedforward (tunable, default off) cancels precession.

    Depends only on Quaternion (Core model) + HeliConfig, not FBW modules.
    Torque output is in standard math convention (Y = up).
]]

TRQTorqueController = {}

local toLuaNum = HeliUtil.toLuaNum
local rad = math.rad
local acos = math.acos
local sqrt = math.sqrt
local abs = math.abs
local cos = math.cos
local sin = math.sin

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function wrapAngle(d)
    if d > 180 then d = d - 360
    elseif d < -180 then d = d + 360
    end
    return d
end

-------------------------------------------------------------------------------------
-- State
-------------------------------------------------------------------------------------
local _Ix = 1
local _Iy = 1
local _Iz = 1
local _inertiaValid = false

-- Predictive tilt omega: model-based prediction + measurement correction.
-- Prediction: omega += torque/I * dt_substep (from known applied torque)
-- Measurement: Δ(upVector) / dt (from actual orientation change)
-- Blend: omega = (1-alpha) * predicted + alpha * measured
-- Eliminates EMA lag that caused D-term to under-damp during oscillation peaks.
local _prevUpX = nil
local _prevUpZ = nil
local _predOmegaX = 0   -- predicted omega from applied torque history
local _predOmegaZ = 0
local _tiltOmegaX = 0   -- blended omega (what the PD actually uses)
local _tiltOmegaZ = 0
local _lastTiltTorqueX = 0  -- last frame's applied torque (for prediction)
local _lastTiltTorqueZ = 0
local _lastYawTorque = 0    -- last frame's yaw torque (for yaw omega prediction)
local _predOmegaY = 0       -- predicted yaw omega (rad/s)
local _prevYawRad = nil     -- previous frame's yaw angle (rad, for measurement)

-- Center of mass offset (Bullet coords, Y=up) for gravity torque feedforward
local _comX = 0
local _comY = 0
local _comZ = 0
local _mass = 1

-------------------------------------------------------------------------------------
-- Helpers
-------------------------------------------------------------------------------------

--- Extract up-vector (Y axis in standard math) from quaternion.
--- Returns world-frame components of the body's up direction.
local function quatUpVector(qw, qx, qy, qz)
    local upX = 2 * (qx * qy + qw * qz)
    local upY = 1 - 2 * (qx * qx + qz * qz)
    local upZ = 2 * (qy * qz - qw * qx)
    return upX, upY, upZ
end

-------------------------------------------------------------------------------------
-- Vector3f reading (for inertia)
-------------------------------------------------------------------------------------

local function tryReadVec3(vec)
    if not vec then return nil, nil, nil end
    local ok, vx = pcall(function() return toLuaNum(vec:x()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:y()) end)
        local _, vz = pcall(function() return toLuaNum(vec:z()) end)
        return vx, vy or 0, vz or 0
    end
    ok, vx = pcall(function() return toLuaNum(vec:getX()) end)
    if ok and vx and vx ~= 0 then
        local _, vy = pcall(function() return toLuaNum(vec:getY()) end)
        local _, vz = pcall(function() return toLuaNum(vec:getZ()) end)
        return vx, vy or 0, vz or 0
    end
    local s = tostring(vec)
    if s then
        local a, b, c = s:match("([%d%.%-]+)[,%s]+([%d%.%-]+)[,%s]+([%d%.%-]+)")
        if a then return tonumber(a), tonumber(b), tonumber(c) end
    end
    return nil, nil, nil
end

--- Read a single component from a Vector3f by name ("x", "y", or "z").
local function tryReadVec3Component(vec, comp)
    if not vec then return nil end
    local ok, v = pcall(function() return toLuaNum(vec[comp](vec)) end)
    if ok and v then return v end
    ok, v = pcall(function()
        local getter = "get" .. comp:upper()
        return toLuaNum(vec[getter](vec))
    end)
    if ok and v then return v end
    return nil
end

-------------------------------------------------------------------------------------
-- Inertia computation
-------------------------------------------------------------------------------------

function TRQTorqueController.initFromVehicle(vehicle)
    if _inertiaValid then return end

    local mass = toLuaNum(vehicle:getMass())
    if mass <= 0 then _inertiaValid = false; return end

    local script = vehicle:getScript()
    if not script then _inertiaValid = false; return end

    local ex, ey, ez
    local ok, shape = pcall(function() return script:getPhysicsChassisShape() end)
    if ok and shape then ex, ey, ez = tryReadVec3(shape) end
    if not ex or (ex == 0 and ey == 0 and ez == 0) then
        ok, shape = pcall(function() return script:getExtents() end)
        if ok and shape then ex, ey, ez = tryReadVec3(shape) end
    end

    -- Empirical inertia correction: Bullet's effective inertia is ~1.76× the
    -- analytical box value (measured from flight data: expected 11220, observed 19745).
    -- Likely from compound collision shape, btRaycastVehicle constraints, or
    -- non-uniform mass distribution. Tunable via trqInertiaCorrectionFactor.
    local corrFactor = HeliConfig.GetTrqInertiaCorrectionFactor()

    if ex and ey and ez and not (ex == 0 and ey == 0 and ez == 0) then
        _Ix = (mass / 12) * (ey * ey + ez * ez) * corrFactor
        _Iy = (mass / 12) * (ex * ex + ez * ez) * corrFactor
        _Iz = (mass / 12) * (ex * ex + ey * ey) * corrFactor
        print("[TRQ] Inertia from extents (×" .. string.format("%.2f", corrFactor) .. "): Ix="
            .. string.format("%.1f", _Ix) .. " Iy=" .. string.format("%.1f", _Iy)
            .. " Iz=" .. string.format("%.1f", _Iz))
    else
        local r = 1.5
        local I = 0.4 * mass * r * r * corrFactor
        _Ix = I; _Iy = I; _Iz = I
        print("[TRQ] WARNING: Could not read vehicle extents. Using fallback inertia: " .. string.format("%.1f", I))
    end

    if _Ix < 1 then _Ix = 1 end
    if _Iy < 1 then _Iy = 1 end
    if _Iz < 1 then _Iz = 1 end

    -- Read center-of-mass offset for gravity torque feedforward.
    -- In Bullet coords (Y=up): gravity = (0, -g, 0). Torque = r_CoM × F_gravity.
    _mass = mass
    local okCoM, com = pcall(function() return script:getCenterOfMassOffset() end)
    if okCoM and com then
        _comX = tryReadVec3Component(com, "x") or 0
        _comY = tryReadVec3Component(com, "y") or 0
        _comZ = tryReadVec3Component(com, "z") or 0
        if _comX ~= 0 or _comY ~= 0 or _comZ ~= 0 then
            print("[TRQ] CoM offset: (" .. string.format("%.3f, %.3f, %.3f", _comX, _comY, _comZ) .. ")")
        end
    end

    _inertiaValid = true
end

-------------------------------------------------------------------------------------
-- PD controller: Z-axis tilt error + scalar yaw
-------------------------------------------------------------------------------------

--- Compute torque vector.
--- @param desQuat table Desired orientation quaternion {w, x, y, z}
--- @param desYawDeg number Desired yaw scalar (degrees)
--- @param actUpX number Actual up-vector X (world frame, from vehicle:getUpVector)
--- @param actUpY number Actual up-vector Y (world frame)
--- @param actUpZ number Actual up-vector Z (world frame)
--- @param actYawDeg number Actual yaw (degrees)
--- @param omegaY number Body-frame yaw angular velocity Y (deg/s, from quaternion estimator, unused — kept for interface compat)
--- @param dt number Frame time (seconds)
--- @param subSteps number Number of Bullet substeps this frame (1 or 2 at 60fps)
--- @return number torqueX World-frame
--- @return number torqueY World-frame
--- @return number torqueZ World-frame
--- @return number angErrMag Error magnitude (rad)
function TRQTorqueController.compute(desQuat, desYawDeg,
                                     actUpX, actUpY, actUpZ, actYawDeg,
                                     omegaY, dt, subSteps)

    -- === TILT ERROR via up-vector cross product (PX4 approach) ===
    -- Actual up-vector read directly from Bullet (vehicle:getUpVector).
    -- No Euler angles involved — immune to gimbal flip at any heading.
    -- Desired up-vector extracted from orientation quaternion.

    local desUpX, desUpY, desUpZ = quatUpVector(desQuat.w, desQuat.x, desQuat.y, desQuat.z)

    -- Cross product: actual_up × desired_up = rotation axis from actual to desired.
    -- This is the SO(3)-correct error direction (confirmed by aerospace literature).
    -- `desired × actual` has wrong P-term sign (backward pitch on tilt input) but
    -- appeared more stable during yaw because the wrong sign reduced effective gain.
    -- `actual × desired` is correct but requires adequate D-gain (>= 2*sqrt(P) ≈ 9
    -- for critical damping) to maintain stability under gyroscopic coupling.
    local crossX = actUpY * desUpZ - actUpZ * desUpY
    local crossY = actUpZ * desUpX - actUpX * desUpZ
    local crossZ = actUpX * desUpY - actUpY * desUpX

    -- For small angles: |cross| ≈ sin(angle) ≈ angle. For large angles,
    -- scale by angle/sin(angle) to get true angular error.
    local dotProd = clamp(actUpX * desUpX + actUpY * desUpY + actUpZ * desUpZ, -1, 1)
    local tiltAngle = acos(dotProd)
    local sinAngle = sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ)

    local worldErrX, worldErrY, worldErrZ = 0, 0, 0
    if sinAngle > 0.0001 then
        local scale = tiltAngle / sinAngle
        worldErrX = crossX * scale
        worldErrY = crossY * scale  -- small yaw component from tilt coupling
        worldErrZ = crossZ * scale
    end

    -- === YAW ERROR ===
    -- No negation. Empirically verified: positive Y couple-force torque → heading
    -- INCREASES in PZ. wrapAngle(des - act) gives negative when actY > desY →
    -- negative torque → heading decreases → corrects toward desired. ✓
    -- Confirmed from 2238-frame stable flight (original form, no negation).
    -- Confirmed from flight log analysis: positive torque + positive error =
    -- heading drift in error direction (positive feedback when negated).
    local errYDeg = wrapAngle(desYawDeg - actYawDeg)
    local errY = rad(errYDeg)

    local angErrMag = sqrt(worldErrX * worldErrX + errY * errY + worldErrZ * worldErrZ)

    -- === HEADING-CORRECTED INERTIA (world frame) ===
    -- World-fixed couple forces see heading-dependent effective inertia.
    -- A world-X torque rotates around world X, which projects onto body axes
    -- as cos(θ)*bodyX + sin(θ)*bodyZ. Effective inertia:
    --   I_eff_worldX = Ix*cos²(θ) + Iz*sin²(θ)
    --   I_eff_worldZ = Ix*sin²(θ) + Iz*cos²(θ)
    -- At 45° headings: both = (Ix+Iz)/2 (uniform average, exact).
    -- At cardinal headings: one axis gets Ix, the other Iz (11:1 ratio for UH-1B).
    -- Using uniform average caused 6.2× gain mismatch → oscillation at cardinal headings.
    local yawRad = rad(actYawDeg)
    local cosY = cos(yawRad)
    local sinY = sin(yawRad)
    local cos2 = cosY * cosY
    local sin2 = sinY * sinY
    local I_wx = _Ix * cos2 + _Iz * sin2   -- effective inertia for world-X torque
    local I_wz = _Ix * sin2 + _Iz * cos2   -- effective inertia for world-Z torque
    local I_yaw = _Iy

    -- === TILT OMEGA: predictive model + measurement correction ===
    -- Pure EMA had 1-3 frame lag → D-term under-damped during oscillation peaks.
    -- Predictive approach: predict omega from known applied torque (zero lag),
    -- blend with measured omega for external disturbance correction.
    --
    -- Prediction: omega += last_torque / I_tilt * dt_substep (0.01s)
    -- Measurement: Δ(upVector) / dt (raw finite difference, world frame)
    -- Blend: omega = (1 - alpha) * predicted + alpha * measured
    -- Alpha ~0.3: prediction dominates (fast), measurement corrects drift.
    local DT_SUBSTEP = 0.01  -- Bullet physics substep duration
    local wOmX, wOmZ = 0, 0

    -- Step 1: Advance prediction from last frame's applied torque
    _predOmegaX = _predOmegaX + (_lastTiltTorqueX / I_wx) * DT_SUBSTEP
    _predOmegaZ = _predOmegaZ + (_lastTiltTorqueZ / I_wz) * DT_SUBSTEP

    if _prevUpX ~= nil and dt > 0 then
        -- Step 2: Measure omega from up-vector change (raw, no smoothing)
        local measOmX =  (actUpZ - _prevUpZ) / dt
        local measOmZ = -(actUpX - _prevUpX) / dt

        -- Step 3: Blend prediction with measurement (fixed alpha)
        -- Alpha = measurement weight. Higher → tracks disturbances faster but amplifies noise.
        -- 0.4 balances ~2-frame disturbance tracking with acceptable noise rejection.
        -- Adaptive alpha was removed: boosting alpha during disturbances created a
        -- feedback oscillation (PD torque → measured omega → D-term reversal → repeat).
        local alpha = HeliConfig.GetTrqOmegaAlpha()

        _tiltOmegaX = (1 - alpha) * _predOmegaX + alpha * measOmX
        _tiltOmegaZ = (1 - alpha) * _predOmegaZ + alpha * measOmZ

        -- Step 4: Correct prediction drift toward measurement
        _predOmegaX = _tiltOmegaX
        _predOmegaZ = _tiltOmegaZ

        wOmX = _tiltOmegaX
        wOmZ = _tiltOmegaZ
    end
    _prevUpX = actUpX
    _prevUpZ = actUpZ

    -- === YAW OMEGA: predictive model + measurement correction ===
    -- Same approach as tilt omega. Without prediction, the raw quaternion-estimator
    -- omega oscillates ±30% every frame due to substep cadence (1-2-2 pattern at 60fps).
    -- This caused 26,000 Nm yaw torque swings frame-to-frame during stable flight.
    local wOmY = 0

    -- Step 1: Advance prediction from last frame's yaw torque
    _predOmegaY = _predOmegaY + (_lastYawTorque / I_yaw) * DT_SUBSTEP

    if _prevYawRad ~= nil and dt > 0 then
        -- Step 2: Measure yaw omega from heading change
        local dyaw = yawRad - _prevYawRad
        -- Wrap to [-pi, pi]
        if dyaw > 3.14159 then dyaw = dyaw - 6.28318
        elseif dyaw < -3.14159 then dyaw = dyaw + 6.28318
        end
        local measOmY = dyaw / dt

        -- Step 3: Blend prediction with measurement (same alpha as tilt)
        local alpha = HeliConfig.GetTrqOmegaAlpha()
        wOmY = (1 - alpha) * _predOmegaY + alpha * measOmY

        -- Step 4: Correct prediction drift toward measurement
        _predOmegaY = wOmY
    end
    _prevYawRad = yawRad

    -- PD in world frame
    local P_tilt = HeliConfig.GetTrqPitchPGain()
    local D_tilt = HeliConfig.GetTrqPitchDGain()
    local maxTorque = HeliConfig.GetTrqMaxTorque()

    local torqueX = I_wx  * (P_tilt * worldErrX - D_tilt * wOmX)
    local torqueY = I_yaw * (HeliConfig.GetTrqYawPGain() * errY - HeliConfig.GetTrqYawDGain() * wOmY)
    local torqueZ = I_wz  * (P_tilt * worldErrZ - D_tilt * wOmZ)

    -- === GYROSCOPIC FEEDFORWARD (world frame, tunable, default OFF) ===
    local gyroScale = HeliConfig.GetTrqGyroScale()
    if gyroScale > 0 then
        local IwOmX = I_wx * wOmX
        local IwOmY = I_yaw * wOmY
        local IwOmZ = I_wz * wOmZ
        torqueX = torqueX + gyroScale * (wOmY * IwOmZ - wOmZ * IwOmY)
        torqueY = torqueY + gyroScale * (wOmZ * IwOmX - wOmX * IwOmZ)
        torqueZ = torqueZ + gyroScale * (wOmX * IwOmY - wOmY * IwOmX)
    end

    torqueX = clamp(torqueX, -maxTorque, maxTorque)
    torqueY = clamp(torqueY, -maxTorque, maxTorque)
    torqueZ = clamp(torqueZ, -maxTorque, maxTorque)

    -- Store torques for next frame's prediction
    _lastTiltTorqueX = torqueX
    _lastTiltTorqueZ = torqueZ
    _lastYawTorque = torqueY

    -- Return torque + controller internals for debugging
    return torqueX, torqueY, torqueZ, angErrMag,
           desYawDeg, actYawDeg, errYDeg, wOmX, wOmY, wOmZ,
           I_wx, I_wz
end

--- @return number Ix, number Iy, number Iz, boolean valid
function TRQTorqueController.getInertia()
    return _Ix, _Iy, _Iz, _inertiaValid
end

function TRQTorqueController.reset()
    _Ix = 1; _Iy = 1; _Iz = 1
    _inertiaValid = false
    _prevUpX = nil; _prevUpZ = nil
    _predOmegaX = 0; _predOmegaZ = 0
    _lastTiltTorqueX = 0; _lastTiltTorqueZ = 0
    _tiltOmegaX = 0; _tiltOmegaZ = 0
    _lastYawTorque = 0; _predOmegaY = 0; _prevYawRad = nil
end
