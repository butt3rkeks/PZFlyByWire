--[[
    HEFSandboxOptions — Sandbox option definitions owned by an engine
]]

--- @class HEFSandboxOption
--- @field field string Sandbox field name (e.g. "GravityEstimate")
--- @field type string "double"|"integer"|"boolean"|"enum"
--- @field default number|boolean Default value
--- @field min number|nil Min (double/integer only)
--- @field max number|nil Max (double/integer only)
--- @field numValues number|nil Enum count (enum only)
--- @field desc string Human-readable description

HEFSandboxOption = {}

--- @param field string
--- @param type string
--- @param default number|boolean
--- @param desc string
--- @param min number|nil
--- @param max number|nil
--- @param numValues number|nil
--- @return HEFSandboxOption
function HEFSandboxOption.new(field, type, default, desc, min, max, numValues)
    return {
        field = field,
        type = type,
        default = default,
        desc = desc,
        min = min,
        max = max,
        numValues = numValues,
    }
end

--- @class HEFSandboxOptions
--- @field namespace string Sandbox namespace owned by this engine (e.g. "FBW")
--- @field options HEFSandboxOption[]

HEFSandboxOptions = {}

--- @param namespace string
--- @param options HEFSandboxOption[]
--- @return HEFSandboxOptions
function HEFSandboxOptions.new(namespace, options)
    return {
        namespace = namespace,
        options = options,
    }
end
