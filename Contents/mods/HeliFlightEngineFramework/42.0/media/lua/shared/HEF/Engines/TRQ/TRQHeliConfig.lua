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
    -- Cascaded rate controller:
    -- Outer loop: position error → bounded rate command
    -- Inner loop: rate tracking with PI (proportional + integral)
    -- Integral handles persistent disturbances (phantom torque during descent)
    trqOuterPGain  = { default = 5.0,  min = 0.1, max = 50.0,  desc = "Outer loop: position error → rate command gain" },
    trqMaxRate     = { default = 2.0,  min = 0.1, max = 10.0,  desc = "Max commanded angular rate (rad/s)" },
    trqInnerPGain  = { default = 20.0, min = 0.1, max = 100.0, desc = "Inner loop: rate error P gain" },
    trqInnerIGain  = { default = 5.0,  min = 0.0, max = 50.0,  desc = "Inner loop: integral gain (disturbance rejection)" },
    trqMaxIntegral = { default = 3.0,  min = 0.1, max = 20.0,  desc = "Anti-windup clamp on integral" },
    -- Legacy PD gains (kept for reference, used by yaw outer loop P)
    trqPitchPGain  = { default = 33.0, min = 0.1, max = 500.0, desc = "Legacy pitch P (unused by rate controller)" },
    trqPitchDGain  = { default = 15.0, min = 0.0, max = 100.0, desc = "Legacy pitch D (unused by rate controller)" },
    trqRollPGain   = { default = 33.0, min = 0.1, max = 500.0, desc = "Legacy roll P (unused)" },
    trqRollDGain   = { default = 15.0, min = 0.0, max = 100.0, desc = "Legacy roll D (unused)" },
    trqYawPGain    = { default = 5.0,  min = 0.1, max = 50.0,  desc = "Yaw outer loop P gain" },
    trqYawDGain    = { default = 10.0, min = 0.0, max = 100.0, desc = "Legacy yaw D (unused)" },

    -- Couple-force geometry
    trqCoupleOffset = { default = 1.0,  min = 0.1, max = 5.0,    desc = "Couple-force offset distance (meters)" },

    -- Angular velocity estimation
    -- Predictive omega blend: 0 = 100% prediction (fast, no lag, drifts without correction),
    -- 1 = 100% measurement (laggy but tracks external disturbances).
    -- 0.3 = prediction dominates for our own torque response, measurement corrects drift.
    trqOmegaAlpha   = { default = 0.4,  min = 0.0, max = 1.0,    desc = "Omega blend: measurement weight (0=predict, 1=measure)" },

    -- Safety limits
    trqMaxTorque    = { default = 200000.0, min = 100.0, max = 500000.0, desc = "Max torque budget (Nm), allocated with tilt priority" },

    -- Gyroscopic feedforward scale (0=disabled, 1=full cancellation)
    trqGyroScale    = { default = 0.0, min = 0.0, max = 2.0, desc = "Gyroscopic feedforward scale (0=off, 1=full)" },

    -- Inertia correction factor. Initial measurement (1.76×) was wrong due to
    -- EMA smoothing lag in omega estimator. Corrected measurement: ~0.88×.
    -- Default 1.0 = use analytical box inertia as-is. Tune empirically if needed.
    trqInertiaCorrectionFactor = { default = 1.0, min = 0.5, max = 5.0, desc = "Multiply analytical inertia by this (empirical correction)" },

    -- Sim inertia factor: the reference sim advances slower for torque because
    -- tilt response is gradual (torque + inertia), not instant (setAngles).
    trqSimInertiaFactor = { default = 2.0, min = 0.5, max = 10.0, desc = "Sim inertia multiplier (tilt response delay)" },

    -- Horizontal correction PD — torque-appropriate values.
    -- FBW uses VEL_FORCE_FACTOR=100 which produces 500kN+ correction forces from small
    -- position errors. With torque the helicopter can't respond instantly → gets launched.
    -- TRQ uses much lower values: gentle correction that doesn't overpower the torque PD.
    trqVelForceFactor       = { default = 0.0,  min = 0.0, max = 100.0, desc = "Correction force scaling (0=disabled, FBW=100)" },
    trqPdErrorThreshold     = { default = 0.5,  min = 0.0, max = 5.0,   desc = "Error threshold for PD vs damping mode (m)" },
    trqFaOffDeadzone        = { default = 1.5,  min = 0.0, max = 10.0,  desc = "FA-off deadzone speed (m/s)" },
    trqFaOffMinDamping      = { default = 0.05, min = 0.0, max = 1.0,   desc = "FA-off minimum damping speed (m/s)" },

    -- Tilt-to-velocity pipeline
    trqTiltNoiseFloor       = { default = 0.005, min = 0.0, max = 0.1,  desc = "Minimum tilt for thrust (rad, FBW=0.0044)" },
    trqNoInputSpeedThreshold = { default = 2.0,  min = 0.0, max = 10.0, desc = "Speed below which no-input flag sets (m/s)" },
    trqDualPathErrorThreshold = { default = 0.5, min = 0.0, max = 5.0,  desc = "Error threshold for dual-path activation (m)" },
    trqDualPathSpeedThreshold = { default = 0.1, min = 0.0, max = 5.0,  desc = "Speed threshold for dual-path activation (m/s)" },

    -- Ground mode
    trqGroundVelocityKill   = { default = 100.0, min = 0.0, max = 500.0, desc = "Ground velocity kill factor" },
    trqGroundVelocityThreshold = { default = 0.01, min = 0.0, max = 1.0, desc = "Ground velocity threshold (m/s)" },
    trqLandingZoneHeight    = { default = 1.0,  min = 0.0, max = 5.0,   desc = "Landing zone height (m)" },
    trqLandingMinSpeedFactor = { default = 0.3, min = 0.0, max = 1.0,   desc = "Landing minimum speed factor" },

    -- Warmup
    trqWarmupFrames = { default = 20, min = 1, max = 120, desc = "Warmup frames before torque control activates" },
}

local TRQ_PARAM_ORDER = {
    "trqOuterPGain", "trqMaxRate", "trqInnerPGain", "trqInnerIGain", "trqMaxIntegral",
    "trqPitchPGain", "trqPitchDGain", "trqRollPGain", "trqRollDGain",
    "trqYawPGain", "trqYawDGain",
    "trqCoupleOffset", "trqOmegaAlpha", "trqMaxTorque", "trqGyroScale",
    "trqInertiaCorrectionFactor", "trqSimInertiaFactor",
    "trqVelForceFactor", "trqPdErrorThreshold", "trqFaOffDeadzone", "trqFaOffMinDamping",
    "trqTiltNoiseFloor", "trqNoInputSpeedThreshold",
    "trqDualPathErrorThreshold", "trqDualPathSpeedThreshold",
    "trqGroundVelocityKill", "trqGroundVelocityThreshold",
    "trqLandingZoneHeight", "trqLandingMinSpeedFactor",
    "trqWarmupFrames",
}

HeliConfig.registerParams(TRQ_PARAMS, TRQ_PARAM_ORDER)

-------------------------------------------------------------------------------------
-- Typed getters: TRQ params. Defined on HeliConfig for uniform access.
-------------------------------------------------------------------------------------

--- @return number Pitch proportional gain (inertia-normalized)
function HeliConfig.GetTrqOuterPGain() return HeliConfig.get("trqOuterPGain") end
function HeliConfig.GetTrqMaxRate() return HeliConfig.get("trqMaxRate") end
function HeliConfig.GetTrqInnerPGain() return HeliConfig.get("trqInnerPGain") end
function HeliConfig.GetTrqInnerIGain() return HeliConfig.get("trqInnerIGain") end
function HeliConfig.GetTrqMaxIntegral() return HeliConfig.get("trqMaxIntegral") end
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
--- @return number Sim inertia multiplier for tilt response delay
function HeliConfig.GetTrqSimInertiaFactor() return HeliConfig.get("trqSimInertiaFactor") end
--- @return number Warmup frames
function HeliConfig.GetTrqVelForceFactor() return HeliConfig.get("trqVelForceFactor") end
function HeliConfig.GetTrqPdErrorThreshold() return HeliConfig.get("trqPdErrorThreshold") end
function HeliConfig.GetTrqFaOffDeadzone() return HeliConfig.get("trqFaOffDeadzone") end
function HeliConfig.GetTrqFaOffMinDamping() return HeliConfig.get("trqFaOffMinDamping") end
function HeliConfig.GetTrqTiltNoiseFloor() return HeliConfig.get("trqTiltNoiseFloor") end
function HeliConfig.GetTrqNoInputSpeedThreshold() return HeliConfig.get("trqNoInputSpeedThreshold") end
function HeliConfig.GetTrqDualPathErrorThreshold() return HeliConfig.get("trqDualPathErrorThreshold") end
function HeliConfig.GetTrqDualPathSpeedThreshold() return HeliConfig.get("trqDualPathSpeedThreshold") end
function HeliConfig.GetTrqGroundVelocityKill() return HeliConfig.get("trqGroundVelocityKill") end
function HeliConfig.GetTrqGroundVelocityThreshold() return HeliConfig.get("trqGroundVelocityThreshold") end
function HeliConfig.GetTrqLandingZoneHeight() return HeliConfig.get("trqLandingZoneHeight") end
function HeliConfig.GetTrqLandingMinSpeedFactor() return HeliConfig.get("trqLandingMinSpeedFactor") end
function HeliConfig.GetTrqWarmupFrames() return HeliConfig.get("trqWarmupFrames") end
