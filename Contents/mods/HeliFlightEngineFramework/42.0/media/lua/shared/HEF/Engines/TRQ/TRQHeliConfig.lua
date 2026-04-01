--[[
    TRQHeliConfig — TRQ engine parameter definitions + typed getters

    Registers TRQ-specific params with HeliConfig at load time.
    Reuses FBW params for gravity, vertical gain, brake/accel/decel, speeds,
    and horizontal PD — only angular PD and couple-force params are TRQ-specific.
]]

-------------------------------------------------------------------------------------
-- TRQ param definitions: registered with HeliConfig at file scope.
-------------------------------------------------------------------------------------

local TRQ_PARAMS = {
    -- Angular PD gains (inertia-normalized: rad/s² per rad error)
    -- Gains compensate for substep reduction (~60% effective at 60fps).
    -- With predictive omega (zero lag), D can be higher without self-amplification.
    -- D=15 (eff≈9, ζ≈1.0) gives near-critical damping for fast convergence.
    trqPitchPGain  = { default = 33.0, min = 0.1, max = 500.0, desc = "Pitch P gain (rad/s² per rad error, inertia-normalized)" },
    trqPitchDGain  = { default = 15.0, min = 0.0, max = 100.0, desc = "Pitch D gain (damping, inertia-normalized)" },
    trqRollPGain   = { default = 33.0, min = 0.1, max = 500.0, desc = "Roll P gain (rad/s² per rad error, inertia-normalized)" },
    trqRollDGain   = { default = 15.0, min = 0.0, max = 100.0, desc = "Roll D gain (damping, inertia-normalized)" },
    trqYawPGain    = { default = 25.0, min = 0.1, max = 500.0, desc = "Yaw P gain (rad/s² per rad error, inertia-normalized)" },
    trqYawDGain    = { default = 10.0, min = 0.0, max = 100.0, desc = "Yaw D gain (damping, inertia-normalized)" },

    -- Couple-force geometry
    trqCoupleOffset = { default = 1.0,  min = 0.1, max = 5.0,    desc = "Couple-force offset distance (meters)" },

    -- Angular velocity estimation
    -- Predictive omega blend: 0 = 100% prediction (fast, no lag, drifts without correction),
    -- 1 = 100% measurement (laggy but tracks external disturbances).
    -- 0.3 = prediction dominates for our own torque response, measurement corrects drift.
    trqOmegaAlpha   = { default = 0.4,  min = 0.0, max = 1.0,    desc = "Omega blend: measurement weight (0=predict, 1=measure)" },

    -- Safety limits
    trqMaxTorque    = { default = 50000.0, min = 100.0, max = 500000.0, desc = "Max torque per axis (Nm)" },

    -- Gyroscopic feedforward scale (0=disabled, 1=full cancellation)
    trqGyroScale    = { default = 0.0, min = 0.0, max = 2.0, desc = "Gyroscopic feedforward scale (0=off, 1=full)" },

    -- Inertia correction factor. Initial measurement (1.76×) was wrong due to
    -- EMA smoothing lag in omega estimator. Corrected measurement: ~0.88×.
    -- Default 1.0 = use analytical box inertia as-is. Tune empirically if needed.
    trqInertiaCorrectionFactor = { default = 1.0, min = 0.5, max = 5.0, desc = "Multiply analytical inertia by this (empirical correction)" },

    -- Warmup
    trqWarmupFrames = { default = 20, min = 1, max = 120, desc = "Warmup frames before torque control activates" },
}

local TRQ_PARAM_ORDER = {
    "trqPitchPGain", "trqPitchDGain", "trqRollPGain", "trqRollDGain",
    "trqYawPGain", "trqYawDGain",
    "trqCoupleOffset", "trqOmegaAlpha", "trqMaxTorque", "trqGyroScale",
    "trqInertiaCorrectionFactor",
    "trqWarmupFrames",
}

HeliConfig.registerParams(TRQ_PARAMS, TRQ_PARAM_ORDER)

-------------------------------------------------------------------------------------
-- Typed getters: TRQ params. Defined on HeliConfig for uniform access.
-------------------------------------------------------------------------------------

--- @return number Pitch proportional gain (inertia-normalized)
function HeliConfig.GetTrqPitchPGain() return HeliConfig.get("trqPitchPGain") end
--- @return number Pitch derivative gain (inertia-normalized)
function HeliConfig.GetTrqPitchDGain() return HeliConfig.get("trqPitchDGain") end
--- @return number Roll proportional gain (inertia-normalized)
function HeliConfig.GetTrqRollPGain() return HeliConfig.get("trqRollPGain") end
--- @return number Roll derivative gain (inertia-normalized)
function HeliConfig.GetTrqRollDGain() return HeliConfig.get("trqRollDGain") end
--- @return number Yaw proportional gain (inertia-normalized)
function HeliConfig.GetTrqYawPGain() return HeliConfig.get("trqYawPGain") end
--- @return number Yaw derivative gain (inertia-normalized)
function HeliConfig.GetTrqYawDGain() return HeliConfig.get("trqYawDGain") end
--- @return number Couple-force offset distance (meters)
function HeliConfig.GetTrqCoupleOffset() return HeliConfig.get("trqCoupleOffset") end
--- @return number Angular velocity EMA smoothing alpha
function HeliConfig.GetTrqOmegaAlpha() return HeliConfig.get("trqOmegaAlpha") end
--- @return number Max torque per axis
function HeliConfig.GetTrqMaxTorque() return HeliConfig.get("trqMaxTorque") end
--- @return number Gyroscopic feedforward scale (0=off, 1=full)
function HeliConfig.GetTrqGyroScale() return HeliConfig.get("trqGyroScale") end
--- @return number Inertia correction factor (Bullet effective / analytical)
function HeliConfig.GetTrqInertiaCorrectionFactor() return HeliConfig.get("trqInertiaCorrectionFactor") end
--- @return number Warmup frames
function HeliConfig.GetTrqWarmupFrames() return HeliConfig.get("trqWarmupFrames") end
