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
    trqPitchPGain  = { default = 20.0, min = 0.1, max = 500.0, desc = "Pitch P gain (rad/s² per rad error, inertia-normalized)" },
    trqPitchDGain  = { default = 6.0,  min = 0.0, max = 100.0, desc = "Pitch D gain (damping, inertia-normalized)" },
    trqRollPGain   = { default = 20.0, min = 0.1, max = 500.0, desc = "Roll P gain (rad/s² per rad error, inertia-normalized)" },
    trqRollDGain   = { default = 6.0,  min = 0.0, max = 100.0, desc = "Roll D gain (damping, inertia-normalized)" },
    trqYawPGain    = { default = 15.0, min = 0.1, max = 500.0, desc = "Yaw P gain (rad/s² per rad error, inertia-normalized)" },
    trqYawDGain    = { default = 5.0,  min = 0.0, max = 100.0, desc = "Yaw D gain (damping, inertia-normalized)" },

    -- Couple-force geometry
    trqCoupleOffset = { default = 1.0,  min = 0.1, max = 5.0,    desc = "Couple-force offset distance (meters)" },

    -- Angular velocity estimation
    trqOmegaAlpha   = { default = 0.4,  min = 0.05, max = 1.0,   desc = "Angular velocity EMA smoothing (0=smooth, 1=raw)" },

    -- Safety limits
    trqMaxTorque    = { default = 200.0, min = 10.0, max = 2000.0, desc = "Max torque per axis (Nm)" },

    -- Warmup
    trqWarmupFrames = { default = 20, min = 1, max = 120, desc = "Warmup frames before torque control activates" },
}

local TRQ_PARAM_ORDER = {
    "trqPitchPGain", "trqPitchDGain", "trqRollPGain", "trqRollDGain",
    "trqYawPGain", "trqYawDGain",
    "trqCoupleOffset", "trqOmegaAlpha", "trqMaxTorque", "trqWarmupFrames",
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
--- @return number Warmup frames
function HeliConfig.GetTrqWarmupFrames() return HeliConfig.get("trqWarmupFrames") end
