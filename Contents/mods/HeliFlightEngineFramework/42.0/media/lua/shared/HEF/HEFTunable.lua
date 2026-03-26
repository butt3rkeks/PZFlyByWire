--[[
    HEFTunable — A single tunable parameter exposed by an engine
]]

--- @class HEFTunable
--- @field name string Short identifier (e.g. "pgain")
--- @field label string Human-readable label (e.g. "P Gain")
--- @field value number Current value
--- @field min number Minimum allowed value
--- @field max number Maximum allowed value
--- @field default number Default value

HEFTunable = {}

--- @param name string
--- @param label string
--- @param value number
--- @param min number
--- @param max number
--- @param default number
--- @return HEFTunable
function HEFTunable.new(name, label, value, min, max, default)
    return {
        name = name,
        label = label,
        value = value,
        min = min,
        max = max,
        default = default,
    }
end
