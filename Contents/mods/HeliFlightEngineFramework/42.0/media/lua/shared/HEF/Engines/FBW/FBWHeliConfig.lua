--[[
    FBWHeliConfig — FBW engine parameter definitions + typed getters

    Registers FBW-specific params with HeliConfig at load time.
    All FBW param definitions and getters live here — HeliConfig.lua only
    contains framework (HEF.*) params.

    Getters are defined as explicit functions on HeliConfig for IDE autocomplete.
    A second engine would create its own XxxHeliConfig.lua with its own params.
]]

-------------------------------------------------------------------------------------
-- FBW param definitions: registered with HeliConfig at file scope.
-- Sandbox-options.txt must match these defaults for sandbox-tunable params.
-------------------------------------------------------------------------------------

local FBW_PARAMS = {
    -- Sandbox-tunable (FBW.* namespace, persisted per-save)
    gravity     = { ns = "FBW", field = "GravityEstimate",     default = 9.8,   min = 5.0,   max = 20.0,   desc = "Gravity (Bullet units/s^2)" },
    engineDeadCondition = { ns = "FBW", field = "EngineDeadCondition", default = 10,    min = 0,     max = 100,    desc = "Engine condition % for engine-dead" },
    engineDeadFallSpeed = { ns = "FBW", field = "EngineDeadFallSpeed", default = 35.0,  min = 3.0,   max = 60.0,   desc = "Engine-dead fall speed (Bullet Y/s)" },
    verticalGain        = { ns = "FBW", field = "ResponsivenessGain",  default = 8.0,   min = 1.0,   max = 40.0,   desc = "Vertical PD base gain (adaptive multiplier auto-tunes)" },
    brake       = { ns = "FBW", field = "BrakingMultiplier",   default = 0.05,  min = 0.01,  max = 1.0,    desc = "Base inertia rate (0.05=1s, 0.10=0.5s)" },
    accel       = { ns = "FBW", field = "AccelMultiplier",     default = 1.5,   min = 0.1,   max = 10.0,   desc = "Acceleration inertia multiplier on brake" },
    decel       = { ns = "FBW", field = "DecelMultiplier",     default = 2.25,  min = 0.1,   max = 10.0,   desc = "Deceleration inertia multiplier on brake" },
    ascend      = { ns = "FBW", field = "AscendSpeed",         default = 6.0,   min = 1.0,   max = 30.0,   desc = "Ascend speed (Bullet Y/s)" },
    descend     = { ns = "FBW", field = "DescendSpeed",        default = 10.0,  min = 1.0,   max = 25.0,   desc = "Descend speed (Bullet Y/s)" },
    maxHorizontalSpeed = { ns = "FBW", field = "MaxHorizontalSpeed",  default = 90.0,  min = 10.0,  max = 1000.0, desc = "Max horizontal speed (m/s)" },
    yawRotationSpeed   = { ns = "FBW", field = "YawSpeed",            default = 0.7,   min = 0.1,   max = 5.0,    desc = "Yaw rotation speed (deg/frame at target FPS)" },

    -- Runtime-only (session overrides, from /hef commands)
    positionProportionalGain = { default = 7.0,   min = 0.1,  max = 50.0,  desc = "Horizontal position error P gain" },
    positionDerivativeGain   = { default = 0.3,   min = 0.0,  max = 5.0,   desc = "Horizontal position error D gain (damping)" },
    maxPositionError         = { default = 10.0,  min = 1.0,  max = 100.0, desc = "Max position error (tanh saturation, meters)" },
    finalStopDampingGain     = { default = 0.3,   min = 0.0,  max = 2.0,   desc = "Velocity damping strength (0.3=smooth, 0.8=snappy)" },
    yawCorrectionGain        = { default = 0.9,   min = 0.0,  max = 1.0,   desc = "Yaw correction strength (0=none, 1=hard lock)" },
    autoLevelSpeed           = { default = 1.0,   min = 0.1,  max = 5.0,   desc = "Auto-leveling speed multiplier" },
}

-- Display order for /hef show (appended after framework params)
local FBW_PARAM_ORDER = {
    "gravity", "engineDeadCondition", "engineDeadFallSpeed", "verticalGain", "brake", "accel", "decel",
    "ascend", "descend", "maxHorizontalSpeed", "yawRotationSpeed",
    "positionProportionalGain", "positionDerivativeGain", "maxPositionError", "finalStopDampingGain", "yawCorrectionGain", "autoLevelSpeed",
}

HeliConfig.registerParams(FBW_PARAMS, FBW_PARAM_ORDER)

-------------------------------------------------------------------------------------
-- Typed getters: FBW params. String key lives here only.
-- Defined on HeliConfig for uniform access (Lua extension method pattern).
-------------------------------------------------------------------------------------

-- Sandbox-tunable (FBW.*)
--- @return number Gravity (Bullet units/s^2)
function HeliConfig.GetGravity() return HeliConfig.get("gravity") end
--- @return number Engine condition % for engine-dead
function HeliConfig.GetEngineDeadCondition() return HeliConfig.get("engineDeadCondition") end
--- @return number Engine-dead fall speed (Bullet Y/s)
function HeliConfig.GetEngineDeadFallSpeed() return HeliConfig.get("engineDeadFallSpeed") end
--- @return number Vertical PD gain (responsiveness)
function HeliConfig.GetVerticalGain() return HeliConfig.get("verticalGain") end
--- @return number Base inertia rate
function HeliConfig.GetBrake() return HeliConfig.get("brake") end
--- @return number Acceleration inertia multiplier
function HeliConfig.GetAccel() return HeliConfig.get("accel") end
--- @return number Deceleration inertia multiplier
function HeliConfig.GetDecel() return HeliConfig.get("decel") end
--- @return number Ascend speed (Bullet Y/s)
function HeliConfig.GetAscend() return HeliConfig.get("ascend") end
--- @return number Descend speed (Bullet Y/s)
function HeliConfig.GetDescend() return HeliConfig.get("descend") end
--- @return number Max horizontal speed (m/s)
function HeliConfig.GetMaxHorizontalSpeed() return HeliConfig.get("maxHorizontalSpeed") end
--- @return number Yaw rotation speed (deg/frame at target FPS)
function HeliConfig.GetYawRotationSpeed() return HeliConfig.get("yawRotationSpeed") end

-- Runtime-only (session overrides)
--- @return number Horizontal position error P gain
function HeliConfig.GetPositionProportionalGain() return HeliConfig.get("positionProportionalGain") end
--- @return number Horizontal position error D gain (damping)
function HeliConfig.GetPositionDerivativeGain() return HeliConfig.get("positionDerivativeGain") end
--- @return number Max position error (tanh saturation, meters)
function HeliConfig.GetMaxPositionError() return HeliConfig.get("maxPositionError") end
--- @return number Velocity damping strength
function HeliConfig.GetFinalStopDampingGain() return HeliConfig.get("finalStopDampingGain") end
--- @return number Yaw correction strength
function HeliConfig.GetYawCorrectionGain() return HeliConfig.get("yawCorrectionGain") end
--- @return number Auto-leveling speed multiplier
function HeliConfig.GetAutoLevelSpeed() return HeliConfig.get("autoLevelSpeed") end
