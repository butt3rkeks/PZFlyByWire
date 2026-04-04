--[[
    ADRCHeliConfig -- ADRC engine parameter definitions + typed getters

    Registers ADRC-specific params with HeliConfig at load time.
    Reuses FBW params for gravity, vertical gain, ascend/descend speeds,
    yaw rotation speed (via HeliConfig.GetXxx getters defined in FBWHeliConfig).
    Only ADRC-specific params (ESO, couple force, drag, input smoothing) live here.

    Architecture: clean cascaded-loop design (no SimModel, no correction forces).
    Horizontal movement comes from physics (tilt -> gravity decomposition).
    Velocity damping (drag model) provides clean stops when level.
]]

-------------------------------------------------------------------------------------
-- ADRC param definitions: registered with HeliConfig at file scope.
-------------------------------------------------------------------------------------

local ADRC_PARAMS = {
    -- ESO / ADRC bandwidths
    adrcEsoWo       = { default = 20.0,  min = 1.0,   max = 100.0,  desc = "ESO observer bandwidth (rad/s, higher=faster tracking)" },
    adrcWcTilt      = { default = 5.0,   min = 0.1,   max = 20.0,   desc = "ADRC tilt controller bandwidth (rad/s)" },
    adrcWcYaw       = { default = 3.0,   min = 0.1,   max = 15.0,   desc = "ADRC yaw controller bandwidth (rad/s)" },

    -- Couple-force geometry
    adrcCoupleOffset = { default = 1.0,  min = 0.1,  max = 5.0,    desc = "Couple-force offset distance (meters)" },

    -- Safety limits
    adrcMaxTorque   = { default = 200000.0, min = 0.0, max = 500000.0, desc = "Max torque budget (Nm), tilt-priority allocation" },

    -- Input rate smoothing time constant (seconds).
    -- Key inputs command angular rates that ramp through a first-order filter.
    -- Higher = smoother, lower = snappier. 0 = disabled (raw step input).
    adrcInputSmoothingTau = { default = 0.2, min = 0.0, max = 2.0, desc = "Input rate smoothing tau (seconds, 0=off)" },

    -- Tilt decay rate (per second) when no directional input is held.
    -- Desired tilt lerps toward level at this rate. Higher = faster return to level.
    adrcTiltDecayRate = { default = 3.0, min = 0.5, max = 10.0, desc = "Tilt decay rate when no input (per sec)" },

    -- Horizontal velocity damping (drag model).
    -- When helicopter is near-level and no tilt input, applies velocity-proportional
    -- drag force for clean stops. Replaces the sim+correction force pipeline.
    -- Terminal speed = max_horizontal_accel / dragCoeff.
    adrcDragCoeff = { default = 2.0, min = 0.0, max = 10.0, desc = "Horizontal drag coefficient (higher=faster stop, lower top speed)" },

    -- Drag activation: helicopter must be within this tilt angle (degrees) of level
    -- for drag to engage. Outside this, physics handles tilt->velocity naturally.
    adrcDragTiltThreshold = { default = 3.0, min = 0.5, max = 15.0, desc = "Max tilt (degrees) for drag to engage" },

    -- Warmup frames: ESO runs at reduced bandwidth during startup to avoid
    -- transient spike from uninitialized states.
    adrcWarmupFrames = { default = 20, min = 1, max = 120, desc = "Warmup frames before full ADRC authority" },

    -- ESO bandwidth ramp: during warmup, observer bandwidth scales from this
    -- fraction of adrcEsoWo up to full adrcEsoWo.
    adrcWarmupWoFraction = { default = 0.3, min = 0.05, max = 1.0, desc = "ESO bandwidth fraction during warmup (0.3=start at 30%)" },

    -- Inertia correction factor: multiplies analytical box inertia.
    -- 1.0 = use API physicsChassisShape as-is. Tune empirically if needed.
    adrcInertiaCorrFactor = { default = 1.0, min = 0.5, max = 5.0, desc = "Multiply analytical inertia (empirical correction)" },

    -- Substep compensation: multiply couple-force torque by physics substep count.
    -- 0 = off (ESO absorbs the 1/N mismatch via disturbance estimate).
    -- 1 = on (torque x N so one substep delivers full frame's angular impulse).
    adrcSubstepCompensation = { default = 0, min = 0, max = 1, desc = "Substep torque compensation (0=off/safe, 1=on)" },

    -- Max yaw lead: clamp how far desired yaw races ahead of actual (degrees).
    -- Prevents yaw PD reversal when desired-actual exceeds 180.
    adrcMaxYawLead = { default = 45, min = 10, max = 180, desc = "Max desired-actual yaw lead (degrees)" },

    -- Gyroscopic feedforward scale (0=disabled, 1=full cancellation).
    -- PZBullet has no gyroscopic compensation; this adds omega x I*omega feedforward.
    adrcGyroScale = { default = 1.0, min = 0.0, max = 2.0, desc = "Gyroscopic feedforward (0=off, 1=full cancellation)" },

    -- Ground mode
    adrcGroundVelocityKill   = { default = 100.0, min = 0.0, max = 500.0, desc = "Ground velocity kill factor" },
    adrcGroundVelocityThreshold = { default = 0.01, min = 0.0, max = 1.0, desc = "Ground velocity threshold (m/s)" },
    adrcLandingZoneHeight    = { default = 1.0, min = 0.0, max = 5.0, desc = "Landing zone height (Z-levels)" },
    adrcLandingMinSpeedFactor = { default = 0.3, min = 0.0, max = 1.0, desc = "Landing minimum speed factor" },

    -- Diagnostics: inertia measurement mode.
    -- 0 = off (normal ADRC). 1 = pitch pulse. 2 = roll pulse. 3 = yaw pulse.
    adrcDiagPulseAxis = { default = 0, min = 0, max = 3, desc = "Diag: 0=off, 1=pitch, 2=roll, 3=yaw pulse" },
    adrcDiagPulseTorque = { default = 10000, min = 0, max = 200000, desc = "Diag: pulse torque magnitude (Nm)" },
}

local ADRC_PARAM_ORDER = {
    "adrcEsoWo", "adrcWcTilt", "adrcWcYaw",
    "adrcCoupleOffset", "adrcMaxTorque",
    "adrcInputSmoothingTau", "adrcTiltDecayRate",
    "adrcDragCoeff", "adrcDragTiltThreshold",
    "adrcWarmupFrames", "adrcWarmupWoFraction",
    "adrcInertiaCorrFactor", "adrcSubstepCompensation",
    "adrcMaxYawLead", "adrcGyroScale",
    "adrcGroundVelocityKill", "adrcGroundVelocityThreshold",
    "adrcLandingZoneHeight", "adrcLandingMinSpeedFactor",
    "adrcDiagPulseAxis", "adrcDiagPulseTorque",
}

HeliConfig.registerParams(ADRC_PARAMS, ADRC_PARAM_ORDER)

-------------------------------------------------------------------------------------
-- Typed getters: ADRC params. Defined on HeliConfig for uniform access.
-------------------------------------------------------------------------------------

--- @return number ESO observer bandwidth (rad/s)
function HeliConfig.GetAdrcEsoWo() return HeliConfig.get("adrcEsoWo") end
--- @return number ADRC tilt controller bandwidth (rad/s)
function HeliConfig.GetAdrcWcTilt() return HeliConfig.get("adrcWcTilt") end
--- @return number ADRC yaw controller bandwidth (rad/s)
function HeliConfig.GetAdrcWcYaw() return HeliConfig.get("adrcWcYaw") end
--- @return number Couple-force offset distance (meters)
function HeliConfig.GetAdrcCoupleOffset() return HeliConfig.get("adrcCoupleOffset") end
--- @return number Max torque budget (Nm)
function HeliConfig.GetAdrcMaxTorque() return HeliConfig.get("adrcMaxTorque") end
--- @return number Input smoothing tau (seconds, 0=off)
function HeliConfig.GetAdrcInputSmoothingTau() return HeliConfig.get("adrcInputSmoothingTau") end
--- @return number Tilt decay rate (per second)
function HeliConfig.GetAdrcTiltDecayRate() return HeliConfig.get("adrcTiltDecayRate") end
--- @return number Horizontal drag coefficient
function HeliConfig.GetAdrcDragCoeff() return HeliConfig.get("adrcDragCoeff") end
--- @return number Drag tilt threshold (degrees)
function HeliConfig.GetAdrcDragTiltThreshold() return HeliConfig.get("adrcDragTiltThreshold") end
--- @return number Warmup frames
function HeliConfig.GetAdrcWarmupFrames() return HeliConfig.get("adrcWarmupFrames") end
--- @return number Warmup ESO bandwidth fraction
function HeliConfig.GetAdrcWarmupWoFraction() return HeliConfig.get("adrcWarmupWoFraction") end
--- @return number Inertia correction factor
function HeliConfig.GetAdrcInertiaCorrFactor() return HeliConfig.get("adrcInertiaCorrFactor") end
--- @return number 0=off, 1=on
function HeliConfig.GetAdrcSubstepCompensation() return HeliConfig.get("adrcSubstepCompensation") end
--- @return number Max yaw lead (degrees)
function HeliConfig.GetAdrcMaxYawLead() return HeliConfig.get("adrcMaxYawLead") end
--- @return number Gyroscopic feedforward scale
function HeliConfig.GetAdrcGyroScale() return HeliConfig.get("adrcGyroScale") end
--- @return number Ground velocity kill factor
function HeliConfig.GetAdrcGroundVelocityKill() return HeliConfig.get("adrcGroundVelocityKill") end
--- @return number Ground velocity threshold (m/s)
function HeliConfig.GetAdrcGroundVelocityThreshold() return HeliConfig.get("adrcGroundVelocityThreshold") end
--- @return number Landing zone height (Z-levels)
function HeliConfig.GetAdrcLandingZoneHeight() return HeliConfig.get("adrcLandingZoneHeight") end
--- @return number Landing minimum speed factor
function HeliConfig.GetAdrcLandingMinSpeedFactor() return HeliConfig.get("adrcLandingMinSpeedFactor") end
--- @return number Diagnostic pulse axis (0=off, 1=pitch, 2=roll, 3=yaw)
function HeliConfig.GetAdrcDiagPulseAxis() return HeliConfig.get("adrcDiagPulseAxis") end
--- @return number Diagnostic pulse torque (Nm)
function HeliConfig.GetAdrcDiagPulseTorque() return HeliConfig.get("adrcDiagPulseTorque") end
