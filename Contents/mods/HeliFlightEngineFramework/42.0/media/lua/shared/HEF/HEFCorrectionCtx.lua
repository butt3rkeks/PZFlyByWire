--[[
    HEFCorrectionCtx — Framework context for the 0-frame delay correction path

    Built by HeliMove on OnTickEvenPaused, passed to applyCorrectionForces().
    Separate from HEFCtx because the correction path runs at a different frame
    phase (before physics) with different state requirements.

    Velocity is read fresh here — physics has stepped since OnTick, so the
    values differ from HEFCtx. Mass is re-read for consistency.

    Engine authors: read the @class annotation and CTX_FIELDS below.
]]

HEFCorrectionCtx = {}

-------------------------------------------------------------------------------------
-- EmmyLua type annotations
-------------------------------------------------------------------------------------

--- @class HEFCorrectionCtx
--- @field vehicle BaseVehicle The helicopter
--- @field mass number Vehicle mass (Lua number)
--- @field velX number Smoothed horizontal velocity X (m/s, fresh post-physics read)
--- @field velZ number Smoothed horizontal velocity Z (m/s, fresh post-physics read)

-------------------------------------------------------------------------------------
-- Runtime contract (mirrors EmmyLua above, used for validation)
-------------------------------------------------------------------------------------

local _types = {
    BaseVehicle = { label = "BaseVehicle", check = function(v) return type(v) == "userdata" end },
    number      = { label = "number",      check = function(v) return type(v) == "number" end },
}

HEFCorrectionCtx.CTX_FIELDS = {
    { name = "vehicle", type = _types.BaseVehicle, desc = "The helicopter" },
    { name = "mass",    type = _types.number,      desc = "Vehicle mass (Lua number)" },
    { name = "velX",    type = _types.number,      desc = "Smoothed horizontal velocity X (m/s, post-physics)" },
    { name = "velZ",    type = _types.number,      desc = "Smoothed horizontal velocity Z (m/s, post-physics)" },
}

-------------------------------------------------------------------------------------
-- Builder
-------------------------------------------------------------------------------------

--- Build the correction-phase context.
--- @param vehicle BaseVehicle
--- @return HEFCorrectionCtx
function HEFCorrectionCtx.build(vehicle)
    local toLuaNum = HeliUtil.toLuaNum
    local vx, _, vz = HeliVelocityAdapter.getVelocity(vehicle)
    return {
        vehicle = vehicle,
        mass = toLuaNum(vehicle:getMass()),
        velX = toLuaNum(vx),
        velZ = toLuaNum(vz),
    }
end
