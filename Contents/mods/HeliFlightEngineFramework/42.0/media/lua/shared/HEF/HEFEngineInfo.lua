--[[
    HEFEngineInfo — Metadata describing a registered flight engine
]]

--- @class HEFEngineInfo
--- @field name string Engine identifier (e.g. "FBW")
--- @field version string Version string (e.g. "1.0")
--- @field description string Human-readable description

HEFEngineInfo = {}

--- @param name string
--- @param version string
--- @param description string
--- @return HEFEngineInfo
function HEFEngineInfo.new(name, version, description)
    return {
        name = name,
        version = version,
        description = description,
    }
end
