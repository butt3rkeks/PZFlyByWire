--[[
    HeliConfig — Centralized configuration for helicopter physics

    Single source of truth for ALL tuning parameters, including:
    - Sandbox-tunable params (read from SandboxVars with hardcoded fallbacks)
    - Runtime-tunable params (PD gains, yaw gain, auto-level, etc.)
    - Named constants (thresholds, factors, limits)

    Runtime overrides via /hef commands write here. No tunable lives outside HeliConfig.
    All modules read from HeliConfig.get() or HeliConfig.CONSTANT instead of holding
    their own tuning state.
]]

HeliConfig = {}

-------------------------------------------------------------------------------------
-- Parameter definitions: single source of truth for names, defaults, descriptions.
-- Sandbox-options.txt must match these defaults for sandbox-tunable params.
-- Runtime-only params (pgain, dgain, etc.) don't appear in sandbox-options.txt.
-------------------------------------------------------------------------------------
local PARAMS = {
    -- Sandbox-tunable (persisted per-save) — min/max match sandbox-options.txt
    gravity     = { field = "GravityEstimate",     default = 9.8,   min = 5.0,   max = 20.0,   desc = "Gravity (Bullet units/s^2)" },
    kp          = { field = "ResponsivenessGain",  default = 8.0,   min = 1.0,   max = 20.0,   desc = "Vertical PD gain (responsiveness)" },
    brake       = { field = "BrakingMultiplier",   default = 0.05,  min = 0.01,  max = 1.0,    desc = "Base inertia rate (0.05=1s, 0.10=0.5s)" },
    accel       = { field = "AccelMultiplier",     default = 1.5,   min = 0.1,   max = 10.0,   desc = "Acceleration inertia multiplier on brake" },
    decel       = { field = "DecelMultiplier",     default = 2.25,  min = 0.1,   max = 10.0,   desc = "Deceleration inertia multiplier on brake" },
    ascend      = { field = "AscendSpeed",         default = 8.0,   min = 1.0,   max = 30.0,   desc = "Ascend speed (Bullet Y/s)" },
    descend     = { field = "DescendSpeed",        default = 14.0,  min = 1.0,   max = 25.0,   desc = "Descend speed (Bullet Y/s)" },
    fall        = { field = "GravityFallSpeed",    default = 24.5,  min = 2.0,   max = 50.0,   desc = "Engine-off fall (Bullet Y/s)" },
    deadfall    = { field = "EngineDeadFallSpeed",  default = 35.0,  min = 3.0,   max = 60.0,   desc = "Engine-dead fall (Bullet Y/s)" },
    hspeed      = { field = "MaxHorizontalSpeed",  default = 450.0, min = 10.0,  max = 1000.0, desc = "Max horizontal speed (m/s)" },

    -- Runtime-only (session overrides, from /hef commands)
    pgain       = { default = 7.0,   min = 0.1,  max = 50.0, desc = "Horizontal position error P gain" },
    dgain       = { default = 0.3,   min = 0.0,  max = 5.0,  desc = "Horizontal position error D gain (damping)" },
    maxerr      = { default = 10.0,  min = 1.0,  max = 100.0, desc = "Max position error (tanh saturation, meters)" },
    fstopgain   = { default = 0.3,   min = 0.0,  max = 2.0,  desc = "Velocity damping strength (0.3=smooth, 0.8=snappy)" },
    yawgain     = { default = 0.9,   min = 0.0,  max = 1.0,  desc = "Yaw correction strength (0=none, 1=hard lock)" },
    autolevel   = { default = 1.0,   min = 0.1,  max = 5.0,  desc = "Auto-leveling speed multiplier" },
}

-- Ordered list for consistent display in /hef show
local PARAM_ORDER = {
    "gravity", "kp", "brake", "accel", "decel",
    "ascend", "descend", "fall", "deadfall", "hspeed",
    "pgain", "dgain", "maxerr", "fstopgain", "yawgain", "autolevel",
}

-- Runtime overrides (session-only, from /hef commands)
local _overrides = {}

-------------------------------------------------------------------------------------
-- Core accessors
-------------------------------------------------------------------------------------

--- Get a parameter value. Priority: runtime override > SandboxVars > hardcoded default.
--- @param shorthand string Parameter key (e.g., "gravity", "pgain")
--- @return number
function HeliConfig.get(shorthand)
    -- Runtime override takes priority (session-only, from /hef commands)
    if _overrides[shorthand] ~= nil then
        return _overrides[shorthand]
    end
    -- SandboxVars (persisted per-save) — only for params with a field mapping
    local p = PARAMS[shorthand]
    if p and p.field and SandboxVars.FBW then
        local val = SandboxVars.FBW[p.field]
        if val ~= nil then return val end
    end
    -- Hardcoded default
    return p and p.default or 0
end

--- Set a runtime override (session-only, not persisted).
--- Value is clamped to [min, max] if bounds are defined for this parameter.
--- @param shorthand string Parameter key
--- @param value number New value
--- @return number Actual value stored (after clamping)
function HeliConfig.set(shorthand, value)
    local p = PARAMS[shorthand]
    if p then
        if p.min and value < p.min then value = p.min end
        if p.max and value > p.max then value = p.max end
        _overrides[shorthand] = value
    end
    return value
end

--- Reset all runtime overrides to sandbox defaults.
function HeliConfig.resetAll()
    _overrides = {}
end

--- Get the parameter definition table (for HeliDebugCommands display).
--- @return table PARAMS, table PARAM_ORDER
function HeliConfig.getParamDefs()
    return PARAMS, PARAM_ORDER
end

-------------------------------------------------------------------------------------
-- Engine selection
-------------------------------------------------------------------------------------

--- Get the active flight engine name from sandbox settings.
--- String field — defaults to "FBW". Third-party mods can set any registered engine name.
--- @return string Engine name (e.g. "FBW")
function HeliConfig.getEngineName()
    if SandboxVars.HEF and SandboxVars.HEF.FlightEngine then
        local name = SandboxVars.HEF.FlightEngine
        if type(name) == "string" and name ~= "" then
            return name
        end
    end
    return "FBW"
end

-------------------------------------------------------------------------------------
-- Physics constants (shared between ForceAdapter and ForceComputer)
-------------------------------------------------------------------------------------

-- Force scaling: converts position/velocity error to Newtons.
-- = 1.0 / PHYSICS_SUBSTEP = 100. Used by ForceComputer (correction forces)
-- and HeliMove ground path (velocity zeroing).
HeliConfig.VEL_FORCE_FACTOR = 100

-- PD error threshold: above this magnitude (meters), PD correction handles
-- deceleration. Below, velocity damping takes over for final stop.
-- Prevents harsh PD→damping force transition at high speed.
HeliConfig.PD_ERROR_THRESHOLD = 0.5

-------------------------------------------------------------------------------------
-- Flight model constants (not sandbox-tunable, but named for clarity)
-------------------------------------------------------------------------------------

-- Airborne detection: helicopter is "airborne" when curr_z > groundLevel + this margin.
-- Below this margin, the ground path handles velocity zeroing and liftoff.
HeliConfig.AIRBORNE_MARGIN = 0.6

-- Landing zone: descent speed tapers in the last N Z-levels above ground.
-- Floor factor prevents descent from stopping entirely (always makes visible progress).
HeliConfig.LANDING_ZONE_HEIGHT = 1.0
HeliConfig.LANDING_MIN_SPEED_FACTOR = 0.3

-- Wall damage rate: ticks between wall damage applications (~1 second at 60 FPS).
HeliConfig.WALL_DAMAGE_INTERVAL = 60

-- Ground velocity zeroing force multiplier.
HeliConfig.GROUND_VELOCITY_KILL = 100

-- Ground velocity threshold: below this magnitude, no zeroing force needed.
HeliConfig.GROUND_VELOCITY_THRESHOLD = 0.01

-- Maximum flight ceiling (Z-levels). Ascend speed tapers near the ceiling.
HeliConfig.MAX_ALTITUDE = 8

-- Ceiling zone: ascend speed tapers in the last N Z-levels below MAX_ALTITUDE.
HeliConfig.CEILING_ZONE_HEIGHT = 1.0

-- Engine dead threshold: engine condition below this triggers engine-dead fall.
HeliConfig.ENGINE_DEAD_CONDITION = 10

-- Minimum altitude for descent/fuel-off checks. Below this, S-key and no-fuel don't apply.
HeliConfig.MIN_DESCENT_ALTITUDE = 0.4

-- Yaw rotation speed (degrees per frame at target FPS).
HeliConfig.YAW_SPEED = 0.7

-- Yaw re-anchor threshold: if yaw error exceeds this, re-anchor to actual (degrees).
HeliConfig.YAW_REANCHOR_THRESHOLD = 10.0

-- Heading calibration tilt threshold (radians).
HeliConfig.HEADING_CALIBRATION_TILT = math.rad(5)

-- Direction threshold: minimum cosine of tilt angle to produce thrust.
HeliConfig.DIRECTION_COS_THRESHOLD = 0.001

-- Tilt noise floor: minimum total tilt deviation (radians) from neutral to produce
-- horizontal thrust. Below this, tilt is treated as noise. Full thrust ramps in
-- over [floor, floor*2].
HeliConfig.TILT_NOISE_FLOOR = math.rad(0.5)

-- Max thrust lead angle (degrees). Prevents pretzel trajectories during turns.
HeliConfig.MAX_THRUST_LEAD = 30

-- No-input speed threshold (m/s): below this, tilt is considered level.
HeliConfig.NO_INPUT_SPEED_THRESHOLD = 2.0

-- Sim snap threshold (m/s): below this + no input, sim blends toward actual.
HeliConfig.SIM_SNAP_THRESHOLD = 2.0

-- Sim soft anchor blend rate: 0.1 = ~90% convergence in 22 frames.
HeliConfig.SIM_BLEND_RATE = 0.1

-- FA-off deadzone (m/s): below this, free-coast mode allows velocity damping.
HeliConfig.FA_OFF_DEADZONE = 1.5

-- FA-off minimum damping speed (m/s): below this, damping is not applied.
HeliConfig.FA_OFF_MIN_DAMPING_SPEED = 0.05

-- Dual-path activation thresholds
HeliConfig.DUAL_PATH_ERROR_THRESHOLD = 0.5
HeliConfig.DUAL_PATH_SPEED_THRESHOLD = 0.1

-- Black zone warning distance (tiles ahead to check for world boundary).
HeliConfig.BLACK_ZONE_CHECK_DISTANCE = 50

-- Auto-level rate factors: controls how fast tilt returns to neutral after key release.
HeliConfig.AUTO_LEVEL_PITCH_FACTOR = 1.5
HeliConfig.AUTO_LEVEL_ROLL_FACTOR = 2.25

-- Target FPS: the baseline frame rate that fpsMultiplier normalizes against.
HeliConfig.TARGET_FPS = 90

-- Minimum FPS floor: prevents extreme fpsMultiplier values during GC pauses.
HeliConfig.MIN_FPS = 10

-- CarController override: set vehicle max speed high enough that CarController
-- never fights our forces.
HeliConfig.MAX_SPEED_OVERRIDE = 999

-- Warmup frames: after reset, skip correction forces for N frames.
HeliConfig.WARMUP_FRAMES = 10

-- History buffer size for PD error rate computation.
HeliConfig.HISTORY_SIZE = 30
