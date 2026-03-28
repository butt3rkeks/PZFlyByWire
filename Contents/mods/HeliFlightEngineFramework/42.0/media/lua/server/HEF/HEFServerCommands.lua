--[[
    HEFServerCommands — Server-side handler for /hef write commands in MP

    Validates admin access, then broadcasts the change to all clients.
    Only active on the server (dedicated or SP/local host).
    Note: server/ Lua loads on clients too (PZ quirk), so we guard with isServer().
]]

if not isServer() then return end

local MOD_ID = "HEF"

Events.OnClientCommand.Add(function(module, command, player, args)
    if module ~= MOD_ID then return end

    -- Admin check: only admins can change flight params in MP
    local accessLevel = player:getAccessLevel()
    if accessLevel == "" or accessLevel == "None" then
        sendServerCommand(player, MOD_ID, "Error", { message = "Access denied. Admin required." })
        return
    end

    if command == "SetParam" then
        -- args: { param = string, value = number }
        if not args or not args.param or not args.value then
            sendServerCommand(player, MOD_ID, "Error", { message = "Invalid arguments." })
            return
        end
        local param = tostring(args.param)
        local value = tonumber(args.value)
        if not value then
            sendServerCommand(player, MOD_ID, "Error", { message = "Invalid number: " .. tostring(args.value) })
            return
        end
        -- Broadcast to all clients so everyone's HeliConfig stays in sync
        sendServerCommand(MOD_ID, "ApplyParam", {
            param = param,
            value = value,
            who = player:getUsername(),
        })

    elseif command == "SetTunable" then
        -- args: { name = string, value = number }
        if not args or not args.name or not args.value then
            sendServerCommand(player, MOD_ID, "Error", { message = "Invalid arguments." })
            return
        end
        sendServerCommand(MOD_ID, "ApplyTunable", {
            name = tostring(args.name),
            value = tonumber(args.value),
            who = player:getUsername(),
        })

    elseif command == "Reset" then
        sendServerCommand(MOD_ID, "ApplyReset", {
            who = player:getUsername(),
        })

    elseif command == "EngineCmd" then
        -- args: { name = string, cmdArgs = string }
        if not args or not args.name then
            sendServerCommand(player, MOD_ID, "Error", { message = "Invalid arguments." })
            return
        end
        sendServerCommand(MOD_ID, "ApplyEngineCmd", {
            name = tostring(args.name),
            cmdArgs = args.cmdArgs and tostring(args.cmdArgs) or "",
            who = player:getUsername(),
        })

    else
        sendServerCommand(player, MOD_ID, "Error", { message = "Unknown command: " .. tostring(command) })
    end
end)
