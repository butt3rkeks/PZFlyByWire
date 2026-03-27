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
--- @field applyForce fun(fx:number, fy:number, fz:number) Apply physics force (Bullet space, adapter-wrapped)

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
--
-- Reuses a module-local table and closure to avoid per-frame allocations.
-- The closure captures 'vehicle' — it is recreated only when vehicle changes.
-------------------------------------------------------------------------------------

local _cctx = {}
local _cctxCachedVehicle = nil
local _cctxApplyForceFn = nil

--- Build the correction-phase context.
--- @param vehicle BaseVehicle
--- @return HEFCorrectionCtx
function HEFCorrectionCtx.build(vehicle)
    local toLuaNum = HeliUtil.toLuaNum

    -- Rebuild closure if vehicle changed (once per flight session)
    if vehicle ~= _cctxCachedVehicle then
        _cctxCachedVehicle = vehicle
        _cctxApplyForceFn = function(fx, fy, fz)
            HeliForceAdapter.applyForceImmediate(vehicle, fx, fy, fz)
        end
    end

    local vx, _, vz = HeliVelocityAdapter.getVelocity(vehicle)

    _cctx.vehicle = vehicle
    _cctx.mass = toLuaNum(vehicle:getMass())
    _cctx.velX = toLuaNum(vx)
    _cctx.velZ = toLuaNum(vz)
    _cctx.applyForce = _cctxApplyForceFn

    return _cctx
end
