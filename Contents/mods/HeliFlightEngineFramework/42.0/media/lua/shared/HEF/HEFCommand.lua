--[[
    HEFCommand — A chat command exposed by an engine
]]

--- @class HEFCommand
--- @field name string Command identifier (e.g. "recalibrate")
--- @field description string What the command does
--- @field args string Expected arguments format (e.g. "" or "<degrees>")

HEFCommand = {}

--- @param name string
--- @param description string
--- @param args string
--- @return HEFCommand
function HEFCommand.new(name, description, args)
    return {
        name = name,
        description = description,
        args = args,
    }
end
