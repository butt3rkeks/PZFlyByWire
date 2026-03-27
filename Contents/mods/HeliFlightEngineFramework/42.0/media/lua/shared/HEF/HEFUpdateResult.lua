--[[
    HEFUpdateResult — Return type for IHEFEngine:update(ctx)

    Engines MUST return these mandatory fields. Optional fields are consumed
    by the framework for wall damage, telemetry, debug logging, and flight
    data recording. A second engine should populate them for full framework
    integration, but omitting them is safe (nil-guarded in HeliMove).
]]

--- @class HEFUpdateResult
--- --- Mandatory (framework behavior depends on these) ---
--- @field engineDead boolean Engine below death threshold — framework shuts off engine
--- @field dualPathActive boolean True = framework fires applyCorrectionForces this frame
--- @field displaySpeed number km/h for speedometer (vehicle:setSpeedKmHour)
--- --- Framework-consumed (wall damage, telemetry) ---
--- @field isBlockedHit boolean Wall collision occurred this frame (triggers wall damage)
--- @field telemetrySpeed number Ground speed (km/h) for Heli_GlobalSpeed export
--- --- Debug / flight data recorder (nil-safe, consumed only when logging active) ---
--- @field desiredVelX number|nil Desired horizontal velocity X from tilt
--- @field desiredVelZ number|nil Desired horizontal velocity Z from tilt
--- @field simVelX number|nil Simulation model velocity X
--- @field simVelZ number|nil Simulation model velocity Z
--- @field errX number|nil Position error X (sim - actual)
--- @field errZ number|nil Position error Z (sim - actual)
--- @field errRateX number|nil Position error rate X
--- @field errRateZ number|nil Position error rate Z
--- @field targetVelY number|nil Desired vertical velocity
--- @field gravComp boolean|nil Gravity compensation active
--- @field hasHInput boolean|nil Tilt input active
--- @field freeMode boolean|nil Flight assist off (free coast mode)
--- @field noHInput boolean|nil No horizontal input detected

HEFUpdateResult = {}

--- @param engineDead boolean
--- @param dualPathActive boolean
--- @param displaySpeed number
--- @return HEFUpdateResult
function HEFUpdateResult.new(engineDead, dualPathActive, displaySpeed)
    return {
        engineDead = engineDead,
        dualPathActive = dualPathActive,
        displaySpeed = displaySpeed,
    }
end
