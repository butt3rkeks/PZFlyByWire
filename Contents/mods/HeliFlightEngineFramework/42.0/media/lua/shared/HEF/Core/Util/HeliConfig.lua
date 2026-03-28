--[[
    HeliConfig — Centralized configuration for helicopter physics

    Single source of truth for the parameter registry, override system, and constants.
    Framework params (HEF.*) are defined here. Engine params are registered at load
    time via HeliConfig.registerParams() — see FBWHeliConfig.lua for FBW params.

    Runtime overrides via /hef commands write here. No tunable lives outside HeliConfig.
    All modules read from HeliConfig.GetXxx() getters or HeliConfig.CONSTANT instead
    of holding their own tuning state.
]]

HeliConfig = {}

-------------------------------------------------------------------------------------
-- Parameter definitions: framework params only.
-- Engine params are added via registerParams() at load time.
-- Sandbox-options.txt must match these defaults for sandbox-tunable params.
-------------------------------------------------------------------------------------
local PARAMS = {
    -- Framework sandbox-tunable (HEF.* namespace, persisted per-save)
    maxAltitude         = { ns = "HEF", field = "MaxAltitude",         default = 8,    min = 1,    max = 50,     desc = "Max flight ceiling (Z-levels)" },
    warmupFrames        = { ns = "HEF", field = "WarmupFrames",        default = 10,   min = 1,    max = 120,    desc = "Startup delay (frames before flight)" },
    wallDamageInterval  = { ns = "HEF", field = "WallDamageInterval",  default = 60,   min = 10,   max = 600,    desc = "Ticks between wall collision damage" },
    engineOffFallSpeed  = { ns = "HEF", field = "EngineOffFallSpeed",  default = 24.5, min = 2.0,  max = 50.0,   desc = "Engine-off fall speed (Bullet Y/s)" },
    fallPdGain          = { ns = "HEF", field = "FallPDGain",          default = 8.0,  min = 1.0,  max = 20.0,   desc = "Fall PD gain (framework engine-off path)" },
}

-- Ordered list for consistent display in /hef show.
-- registerParams() appends engine entries to this list.
local PARAM_ORDER = {
    "maxAltitude", "warmupFrames", "wallDamageInterval", "engineOffFallSpeed", "fallPdGain",
}

-- Runtime overrides (session-only, from /hef commands)
local _overrides = {}

-------------------------------------------------------------------------------------
-- Core accessors
-------------------------------------------------------------------------------------

--- Get a parameter value. Priority: runtime override > SandboxVars > hardcoded default.
--- @param shorthand string Parameter key (e.g., "gravity", "positionProportionalGain")
--- @return number
function HeliConfig.get(shorthand)
    if shorthand == nil then error("HeliConfig.get: nil key") end
    -- Runtime override takes priority (session-only, from /hef commands)
    if _overrides[shorthand] ~= nil then
        return _overrides[shorthand]
    end
    -- SandboxVars (persisted per-save) — only for params with namespace + field
    local p = PARAMS[shorthand]
    if p and p.ns and p.field then
        local nsTable = SandboxVars[p.ns]
        if nsTable then
            local val = nsTable[p.field]
            if val ~= nil then return val end
        end
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
    if shorthand == nil then error("HeliConfig.set: nil key") end
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

--- Register engine params. Called at file scope by engine config modules
--- (e.g., FBWHeliConfig.lua). Merges param definitions into the shared registry
--- and appends keys to PARAM_ORDER for /hef show display.
--- @param params table Map of shorthand → param definition (same format as PARAMS entries)
--- @param order string[]|nil Optional ordered list of keys for display. If nil, keys are appended in pairs() order.
function HeliConfig.registerParams(params, order)
    for name, def in pairs(params) do
        if PARAMS[name] then
            error("HeliConfig.registerParams: param '" .. name .. "' already registered")
        end
        PARAMS[name] = def
    end
    if order then
        for _, name in ipairs(order) do
            PARAM_ORDER[#PARAM_ORDER + 1] = name
        end
    else
        for name, _ in pairs(params) do
            PARAM_ORDER[#PARAM_ORDER + 1] = name
        end
    end
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

-- Transition zone: engines can blend ground/airborne forces across this altitude band.
-- BOTTOM = pure ground hold. TOP = full airborne authority.
HeliConfig.TRANSITION_ZONE_BOTTOM = 0.1
HeliConfig.TRANSITION_ZONE_TOP    = 0.5

-- Airborne detection: helicopter is "airborne" when currentAltitude > groundLevel + this margin.
-- Set to match TRANSITION_ZONE_TOP so the engine's ground handler owns the entire transition.
HeliConfig.AIRBORNE_MARGIN = HeliConfig.TRANSITION_ZONE_TOP

-- Landing zone: descent speed tapers in the last N Z-levels above ground.
-- Floor factor prevents descent from stopping entirely (always makes visible progress).
HeliConfig.LANDING_ZONE_HEIGHT = 1.0
HeliConfig.LANDING_MIN_SPEED_FACTOR = 0.3

-- Ground velocity zeroing force multiplier.
HeliConfig.GROUND_VELOCITY_KILL = 100

-- Ground velocity threshold: below this magnitude, no zeroing force needed.
HeliConfig.GROUND_VELOCITY_THRESHOLD = 0.01

-- Ceiling zone: ascend speed tapers in the last N Z-levels below MAX_ALTITUDE.
HeliConfig.CEILING_ZONE_HEIGHT = 1.0

-- Minimum altitude for descent/fuel-off checks. Below this, S-key and no-fuel don't apply.
HeliConfig.MIN_DESCENT_ALTITUDE = 0.4

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

-- History buffer size for PD error rate computation.
HeliConfig.HISTORY_SIZE = 30

-------------------------------------------------------------------------------------
-- Typed getters: framework params only. String key lives here only.
-- Engine getters are defined by engine config modules (e.g., FBWHeliConfig.lua).
-- Engines should use HeliConfig.GetXxx() instead of HeliConfig.get("string").
-------------------------------------------------------------------------------------

--- @return number Max flight ceiling (Z-levels)
function HeliConfig.GetMaxAltitude() return HeliConfig.get("maxAltitude") end
--- @return number Startup delay (frames before flight)
function HeliConfig.GetWarmupFrames() return HeliConfig.get("warmupFrames") end
--- @return number Ticks between wall collision damage
function HeliConfig.GetWallDamageInterval() return HeliConfig.get("wallDamageInterval") end
--- @return number Engine-off fall speed (Bullet Y/s)
function HeliConfig.GetEngineOffFallSpeed() return HeliConfig.get("engineOffFallSpeed") end
--- @return number Fall PD gain (framework engine-off path)
function HeliConfig.GetFallPdGain() return HeliConfig.get("fallPdGain") end
