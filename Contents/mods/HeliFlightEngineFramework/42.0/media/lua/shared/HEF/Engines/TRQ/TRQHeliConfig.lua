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
    -- ADRC / Extended State Observer (ESO) parameters
    -- Replaces the cascaded rate controller with direct disturbance estimation.
    -- ESO observes the tilt/yaw error, estimates angular rate AND unknown disturbance
    -- torque in real-time, then cancels the disturbance in the control law.
    -- wo = observer bandwidth (how fast the ESO tracks disturbances)
    -- wc = controller bandwidth (how fast the controller corrects errors)
    -- Rule of thumb: wo = 5-10× wc for good separation.
    trqEsoWo       = { default = 30.0, min = 0.0,  max = 100.0, desc = "ESO observer bandwidth (rad/s, higher=faster disturbance tracking)" },
    trqEsoWcTilt   = { default = 5.0,  min = 0.0,  max = 20.0,  desc = "ADRC tilt controller bandwidth (rad/s)" },
    trqEsoWcYaw    = { default = 3.0,  min = 0.0,  max = 15.0,  desc = "ADRC yaw controller bandwidth (rad/s)" },

    -- Legacy params (kept for reference / fallback)
    trqOuterPGain  = { default = 5.0,  min = 0.1, max = 50.0,  desc = "Legacy: outer loop P gain (unused by ADRC)" },
    trqMaxRate     = { default = 2.0,  min = 0.1, max = 10.0,  desc = "Legacy: max rate (unused by ADRC)" },
    trqInnerPGain  = { default = 20.0, min = 0.1, max = 100.0, desc = "Legacy: inner loop P (unused by ADRC)" },
    trqInnerIGain  = { default = 5.0,  min = 0.0, max = 50.0,  desc = "Legacy: inner loop I (unused by ADRC)" },
    trqMaxIntegral = { default = 3.0,  min = 0.1, max = 20.0,  desc = "Legacy: integral clamp (unused by ADRC)" },
    trqPitchPGain  = { default = 33.0, min = 0.1, max = 500.0, desc = "Legacy pitch P (unused)" },
    trqPitchDGain  = { default = 15.0, min = 0.0, max = 100.0, desc = "Legacy pitch D (unused)" },
    trqRollPGain   = { default = 33.0, min = 0.1, max = 500.0, desc = "Legacy roll P (unused)" },
    trqRollDGain   = { default = 15.0, min = 0.0, max = 100.0, desc = "Legacy roll D (unused)" },
    trqYawPGain    = { default = 5.0,  min = 0.1, max = 50.0,  desc = "Legacy yaw P (unused by ADRC)" },
    trqYawDGain    = { default = 10.0, min = 0.0, max = 100.0, desc = "Legacy yaw D (unused)" },

    -- Couple-force geometry
    trqCoupleOffset = { default = 1.0,  min = 0.1, max = 5.0,    desc = "Couple-force offset distance (meters)" },

    -- Angular velocity estimation
    -- Predictive omega blend: 0 = 100% prediction (fast, no lag, drifts without correction),
    -- 1 = 100% measurement (laggy but tracks external disturbances).
    -- 0.3 = prediction dominates for our own torque response, measurement corrects drift.
    trqOmegaAlpha   = { default = 0.4,  min = 0.0, max = 1.0,    desc = "Omega blend: measurement weight (0=predict, 1=measure)" },

    -- Safety limits
    trqMaxTorque    = { default = 200000.0, min = 0.0, max = 500000.0, desc = "Max torque budget (Nm), allocated with tilt priority" },

    -- Diagnostics: inertia measurement mode.
    -- 0 = off (normal ADRC). 1 = pitch pulse. 2 = roll pulse. 3 = yaw pulse.
    -- When active: disables ESO, applies fixed 10000 Nm torque on selected axis
    -- for 60 frames, then 0 for 60 frames, repeating. Log captures actual angular
    -- rate in the ESO diagnostic columns for inertia calculation.
    trqDiagPulseAxis = { default = 0, min = 0, max = 3, desc = "Diag: 0=off, 1=pitch pulse, 2=roll pulse, 3=yaw pulse" },
    trqDiagPulseTorque = { default = 10000, min = 0, max = 200000, desc = "Diag: pulse torque magnitude (Nm)" },

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

    -- Input rate smoothing time constant (seconds). Key inputs command angular rates
    -- that ramp through a first-order filter instead of stepping instantly.
    -- Higher = smoother (less torque spike on key press/release), lower = snappier.
    -- 0 = disabled (raw step input, causes ±200kNm saturation on every key event).
    trqInputSmoothingTau = { default = 0.2, min = 0.0, max = 2.0, desc = "Input rate smoothing time constant (seconds, 0=off)" },

    -- Substep compensation: multiply applied torque by physics substep count.
    -- 0 = off (ESO absorbs the 1/N mismatch via x3 disturbance estimate).
    -- 1 = on (torque × N so one substep delivers full frame's angular impulse).
    -- Risk: if Lua substep count desyncs from Java's, gain swings 1x↔4x per frame.
    trqSubstepCompensation = { default = 0, min = 0, max = 1, desc = "Substep torque compensation (0=off/safe, 1=on/precise but sync-sensitive)" },
}

local TRQ_PARAM_ORDER = {
    "trqEsoWo", "trqEsoWcTilt", "trqEsoWcYaw",
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
    "trqInputSmoothingTau",
    "trqSubstepCompensation",
}

HeliConfig.registerParams(TRQ_PARAMS, TRQ_PARAM_ORDER)

-------------------------------------------------------------------------------------
-- Typed getters: TRQ params. Defined on HeliConfig for uniform access.
-------------------------------------------------------------------------------------

--- @return number ESO observer bandwidth (rad/s)
function HeliConfig.GetTrqEsoWo() return HeliConfig.get("trqEsoWo") end
--- @return number ADRC tilt controller bandwidth (rad/s)
function HeliConfig.GetTrqEsoWcTilt() return HeliConfig.get("trqEsoWcTilt") end
--- @return number ADRC yaw controller bandwidth (rad/s)
function HeliConfig.GetTrqEsoWcYaw() return HeliConfig.get("trqEsoWcYaw") end
--- @return number Legacy outer P gain
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
function HeliConfig.GetTrqDiagPulseAxis() return HeliConfig.get("trqDiagPulseAxis") end
function HeliConfig.GetTrqDiagPulseTorque() return HeliConfig.get("trqDiagPulseTorque") end
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
--- @return number Input smoothing tau (seconds, 0=off)
function HeliConfig.GetTrqInputSmoothingTau() return HeliConfig.get("trqInputSmoothingTau") end
--- @return number 0=off, 1=on
function HeliConfig.GetTrqSubstepCompensation() return HeliConfig.get("trqSubstepCompensation") end
