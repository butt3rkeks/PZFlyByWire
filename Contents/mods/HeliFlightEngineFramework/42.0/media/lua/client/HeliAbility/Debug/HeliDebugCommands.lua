--[[
    HeliDebugCommands — Runtime tuning via chat commands

    Hooks ISChat.onCommandEntered to intercept /hef commands.
    Auto-discovers tunables from active engine via HeliSimService.getTunables().
    Changes are session-only.

    Networking:
    - SP/local host: all commands execute directly (isServer() is true)
    - Dedicated MP client: write commands go through server for admin validation
      (sendClientCommand → HEFServerCommands → sendServerCommand → OnServerCommand)
    - Read-only commands (help/show/vel/log/snap/record) always run client-local
]]

local ISChat = ISChat
local MOD_ID = "HEF"
local MOD_TAG = "[HEF]"

--- True when we are the authority (SP, local host, or dedicated server process).
--- Write commands execute directly without networking.
local function isAuthority()
    return isServer()
end

--- True when we are a remote MP client that must send write commands to the server.
local function isRemoteClient()
    return not isServer() and isClient()
end

-------------------------------------------------------------------------------------
-- Chat message display
-------------------------------------------------------------------------------------
local function showMessage(message, color)
    if type(color) ~= "string" then
        color = "<RGB:100,200,255>"
    end
    message = color .. " " .. message
    local msg = {
        getText = function(_) return message end,
        getTextWithPrefix = function(_) return message end,
        isServerAlert = function(_) return false end,
        isShowAuthor = function(_) return false end,
        getAuthor = function(_) return "" end,
        setShouldAttractZombies = function(_) return false end,
        setOverHeadSpeech = function(_) return false end,
    }
    if not ISChat.instance then return end
    if not ISChat.instance.chatText then return end
    ISChat.addLineInChat(msg, 0)
end

local function showInfo(message)
    showMessage(MOD_TAG .. " " .. message, "<RGB:200,200,200>")
end

local function showValue(message)
    showMessage(MOD_TAG .. " " .. message, "<RGB:100,200,255>")
end

local function showError(message)
    showMessage(MOD_TAG .. " " .. message, "<RGB:255,80,80>")
end

local function showSuccess(message)
    showMessage(MOD_TAG .. " " .. message, "<RGB:80,255,80>")
end

-------------------------------------------------------------------------------------
-- Write operations: direct in SP, networked in MP
-------------------------------------------------------------------------------------

--- Set a HeliConfig param. In MP, sends to server for admin validation + broadcast.
--- @param shorthand string
--- @param value number
local function doSetParam(shorthand, value)
    if isRemoteClient() then
        sendClientCommand(MOD_ID, "SetParam", { param = shorthand, value = value })
        showInfo("Sent " .. shorthand .. " = " .. tostring(value) .. " to server...")
    else
        local oldVal = HeliConfig.get(shorthand)
        local clamped = HeliConfig.set(shorthand, value)
        showSuccess(shorthand .. ": " .. string.format("%.2f", oldVal) ..
            " -> " .. string.format("%.2f", clamped))
    end
end

--- Set an engine tunable. In MP, sends to server for admin validation + broadcast.
--- @param name string
--- @param value number
local function doSetTunable(name, value)
    if isRemoteClient() then
        sendClientCommand(MOD_ID, "SetTunable", { name = name, value = value })
        showInfo("Sent " .. name .. " = " .. tostring(value) .. " to server...")
    else
        local tunables = HeliSimService.getTunables()
        local oldVal = 0
        if tunables then
            for _, t in ipairs(tunables) do
                if t.name == name then oldVal = t.value; break end
            end
        end
        HeliSimService.setTunable(name, value)
        local newVal = HeliSimService.getTunable(name)
        showSuccess(name .. ": " .. string.format("%.2f", oldVal) ..
            " -> " .. string.format("%.2f", newVal))
    end
end

--- Reset all params and tunables. In MP, sends to server for admin validation + broadcast.
local function doReset()
    if isRemoteClient() then
        sendClientCommand(MOD_ID, "Reset", {})
        showInfo("Sent reset to server...")
    else
        HeliConfig.resetAll()
        local tunables = HeliSimService.getTunables()
        if tunables then
            for _, t in ipairs(tunables) do
                HeliSimService.setTunable(t.name, t.default)
            end
        end
        showSuccess("All parameters reset to defaults.")
    end
end

--- Execute an engine command. In MP, sends to server for admin validation + broadcast.
--- @param name string
--- @param argsStr string
local function doEngineCommand(name, argsStr)
    if isRemoteClient() then
        sendClientCommand(MOD_ID, "EngineCmd", { name = name, cmdArgs = argsStr })
        showInfo("Sent engine command '" .. name .. "' to server...")
    else
        local response = HeliSimService.executeCommand(name, argsStr)
        if response then showInfo(response) end
    end
end

-------------------------------------------------------------------------------------
-- Server response handler (MP only)
-------------------------------------------------------------------------------------

Events.OnServerCommand.Add(function(module, command, args)
    if module ~= MOD_ID then return end

    if command == "ApplyParam" then
        local clamped = HeliConfig.set(args.param, args.value)
        showSuccess(args.param .. " -> " .. string.format("%.2f", clamped) ..
            " (by " .. tostring(args.who) .. ")")

    elseif command == "ApplyTunable" then
        HeliSimService.setTunable(args.name, args.value)
        local newVal = HeliSimService.getTunable(args.name)
        showSuccess(args.name .. " -> " .. string.format("%.2f", newVal) ..
            " (by " .. tostring(args.who) .. ")")

    elseif command == "ApplyReset" then
        HeliConfig.resetAll()
        local tunables = HeliSimService.getTunables()
        if tunables then
            for _, t in ipairs(tunables) do
                HeliSimService.setTunable(t.name, t.default)
            end
        end
        showSuccess("All parameters reset to defaults (by " .. tostring(args.who) .. ")")

    elseif command == "ApplyEngineCmd" then
        local response = HeliSimService.executeCommand(args.name, args.cmdArgs)
        if response then
            showInfo(response .. " (by " .. tostring(args.who) .. ")")
        end

    elseif command == "Error" then
        showError(args.message or "Unknown error")
    end
end)

-------------------------------------------------------------------------------------
-- Command handlers (read-only, always client-local)
-------------------------------------------------------------------------------------

local commands = {}

commands["help"] = function(_)
    local info = HeliSimService.getInfo()
    showInfo("--- HEF Chat Commands (engine: " .. (info and info.name or "?") .. ") ---")
    showInfo("/hef show    — display all current values")
    showInfo("/hef reset   — reset tunables to defaults")
    showInfo("/hef engine [name] — show or switch flight engine")
    showInfo("/hef vel     — show current Bullet velocity")
    showInfo("/hef log     — toggle periodic logging (1/sec to console)")
    showInfo("/hef snap    — one-shot detailed state dump to chat")
    showInfo("/hef record  — toggle flight data CSV recording (per-frame)")
    showInfo("/hef <tunable> <value> — set tunable:")
    local tunables = HeliSimService.getTunables()
    if tunables then
        for _, t in ipairs(tunables) do
            showInfo("  " .. t.name .. " — " .. (t.label or t.name) ..
                " [default: " .. tostring(t.default) .. "]")
        end
    end
    -- Engine-specific commands
    local cmds = HeliSimService.getCommands()
    if cmds and #cmds > 0 then
        showInfo("Engine commands:")
        for _, c in ipairs(cmds) do
            showInfo("  " .. c.name .. " " .. (c.args or "") .. " — " .. (c.description or ""))
        end
    end
    -- Sandbox params
    local PARAMS, PARAM_ORDER = HeliConfig.getParamDefs()
    showInfo("Sandbox params (/hef <param> <value>):")
    for _, shorthand in ipairs(PARAM_ORDER) do
        local p = PARAMS[shorthand]
        showInfo("  " .. shorthand .. " — " .. p.desc .. " [default: " .. tostring(p.default) .. "]")
    end
    if isRemoteClient() then
        showInfo("(MP mode: write commands require admin access)")
    end
end

commands["engine"] = function(args)
    if not args[1] then
        local info = HeliSimService.getInfo()
        local names = IFlightEngine.getRegisteredNames()
        showValue("Active: " .. (info and info.name or "?"))
        showInfo("Available: " .. table.concat(names, ", "))
        return
    end
    local name = args[1]
    local result = HeliSimService.switchEngine(name)
    if result then
        showSuccess("Switched to engine: " .. name)
    else
        showError("Engine '" .. name .. "' not registered.")
        showInfo("Available: " .. table.concat(IFlightEngine.getRegisteredNames(), ", "))
    end
end

commands["show"] = function(_)
    local info = HeliSimService.getInfo()
    showInfo("--- Current HEF Values (engine: " .. (info and info.name or "?") .. ") ---")
    -- Sandbox params
    local PARAMS, PARAM_ORDER = HeliConfig.getParamDefs()
    for _, shorthand in ipairs(PARAM_ORDER) do
        local p = PARAMS[shorthand]
        local val = HeliConfig.get(shorthand)
        showValue("  " .. shorthand .. " = " .. string.format("%.2f", val) ..
            " [default: " .. tostring(p.default) .. "]")
    end
    -- Vehicle diagnostics
    local player = getPlayer()
    local vehicle = player and player:getVehicle()
    if vehicle and vehicle:getScript() then
        local wc = vehicle:getScript():getWheelCount()
        showInfo(string.format("  wheels = %d", wc))
        if wc == 0 then
            showError("  WARNING: 0-wheel vehicle — velocity dampener active (0.1x).")
            showError("  This vehicle was spawned before HEFWheelInjector ran.")
            showError("  Despawn and respawn the helicopter to apply the phantom wheel.")
        end
    end
    -- Engine tunables
    local tunables = HeliSimService.getTunables()
    if tunables then
        local parts = {}
        for _, t in ipairs(tunables) do
            parts[#parts + 1] = t.name .. "=" .. string.format("%.2f", t.value)
        end
        showInfo("  " .. table.concat(parts, "  "))
    end
end

commands["reset"] = function(_)
    doReset()
end

commands["vel"] = function(_)
    local player = getPlayer()
    local vehicle = player and player:getVehicle()
    if not vehicle then
        showError("Not in a vehicle.")
        return
    end
    local vx, vy, vz = HeliVelocityAdapter.getLastVelocity()
    showValue(string.format("Bullet velocity: X=%.3f  Y(height)=%.3f  Z=%.3f",
        HeliUtil.toLuaNum(vx), HeliUtil.toLuaNum(vy), HeliUtil.toLuaNum(vz)))
    showValue(string.format("Mass=%.0f  FPS=%.0f  SubSteps=%.2f",
        vehicle:getMass(), getAverageFPS(),
        1.0 / (math.max(getAverageFPS(), 10) * 0.01)))
end

commands["record"] = function(_)
    if not HeliDebug then
        showError("HeliDebug not available.")
        return
    end
    if HeliDebug.isRecording() then
        local filename = HeliDebug.getRecordingFilename() or "?"
        local count = HeliDebug.stopRecording()
        showSuccess("Recording stopped. " .. count .. " frames saved to " .. filename)
    else
        local filename = HeliDebug.startRecording()
        if filename then
            showSuccess("Recording started: " .. filename)
            showInfo("Fly, then /hef record again to stop. Auto-stops on exit vehicle.")
        else
            showError("Failed to start recording (file write error).")
        end
    end
end

commands["log"] = function(_)
    if not HeliDebug then
        showError("HeliDebug not available.")
        return
    end
    HeliDebug.logEnabled = not HeliDebug.logEnabled
    HeliDebug.lastLogTick = 0
    if HeliDebug.logEnabled then
        showSuccess("Periodic logging ON (check Lua console)")
    else
        showInfo("Periodic logging OFF")
    end
end

commands["snap"] = function(_)
    if not HeliDebug then
        showError("HeliDebug not available.")
        return
    end
    HeliDebug.snapRequested = true
    local s = HeliDebug.lastState
    if not s or not s.state then
        showError("No flight data yet. Fly the helicopter first.")
        return
    end
    showInfo("--- HEF Snapshot ---")
    showValue(string.format("State: %s  |  Z: %.2f  |  Ground: %.2f", s.state or "?", s.currentAltitude or 0, s.groundLevelZ or 0))
    showValue(string.format("Desired vel: X=%.2f  Z=%.2f  |  Sim vel: X=%.2f  Z=%.2f",
        s.desiredVelX or 0, s.desiredVelZ or 0, s.simVelX or 0, s.simVelZ or 0))
    showValue(string.format("Pos error:   X=%.2f  Z=%.2f  |  Rate: X=%.3f  Z=%.3f",
        s.posErrorX or 0, s.posErrorZ or 0, s.posErrorRateX or 0, s.posErrorRateZ or 0))
    showValue(string.format("Vert: savedVelY=%.2f  |  GravComp=%s  EngDead=%s",
        s.savedVelY or 0, s.gravComp and "Y" or "N", s.engineDead and "Y" or "N"))
    showValue(string.format("Blocked=%s  NoHInput=%s  FreeMode=%s  Keys=%s",
        s.blocked and "Y" or "N",
        s.noHInput and "Y" or "N",
        s.freeMode and "Y" or "N",
        s.keys or "-"))
    showValue(string.format("Mass=%.0f  FPS=%.0f  SubSteps=%.2f", s.mass or 0, s.fps or 0, s.subSteps or 0))
    showInfo("--- Active Tuning ---")
    showValue(string.format("gravity=%.1f  verticalGain=%.1f  brake=%.2f  maxHorizontalSpeed=%.1f",
        HeliConfig.GetGravity(), HeliConfig.GetVerticalGain(), HeliConfig.GetBrake(), HeliConfig.GetMaxHorizontalSpeed()))
end

-------------------------------------------------------------------------------------
-- ISChat hook
-------------------------------------------------------------------------------------

local original_onCommandEntered = ISChat["onCommandEntered"]

ISChat["onCommandEntered"] = function(self)
    local commandText = ISChat.instance.textEntry:getText()

    if commandText and commandText ~= "" then
        local words = {}
        for word in commandText:gmatch("%S+") do
            words[#words + 1] = word
        end

        if #words >= 1 and string.lower(words[1]) == "/hef" then
            local subCommand = words[2] and string.lower(words[2]) or "help"
            local rawSubCommand = words[2] or "help"  -- preserve case for param names
            local handler = commands[subCommand]

            if handler then
                local args = {}
                for i = 3, #words do
                    args[#args + 1] = words[i]
                end
                handler(args)
            else
                -- Try as engine tunable (case-insensitive match)
                local tunables = HeliSimService.getTunables()
                local tunableMatch = nil
                local tunableName = nil
                if tunables then
                    for _, t in ipairs(tunables) do
                        if string.lower(t.name) == subCommand then
                            tunableMatch = t
                            tunableName = t.name  -- use canonical name
                            break
                        end
                    end
                end

                if tunableMatch then
                    if not words[3] then
                        showValue(tunableName .. " = " .. string.format("%.2f", tunableMatch.value))
                    else
                        local value = tonumber(words[3])
                        if not value then
                            showError("Invalid number: " .. tostring(words[3]))
                        else
                            doSetTunable(tunableName, value)
                        end
                    end
                else
                    -- Try as HeliConfig param (case-insensitive match)
                    local PARAMS = HeliConfig.getParamDefs()
                    local paramName = nil
                    for name, _ in pairs(PARAMS) do
                        if string.lower(name) == subCommand then
                            paramName = name
                            break
                        end
                    end
                    if paramName then
                        if not words[3] then
                            local val = HeliConfig.get(paramName)
                            showValue(paramName .. " = " .. string.format("%.2f", val))
                        else
                            local value = tonumber(words[3])
                            if not value then
                                showError("Invalid number: " .. tostring(words[3]))
                            else
                                doSetParam(paramName, value)
                            end
                        end
                    else
                        -- Try as engine command (case-insensitive)
                        local engineCmds = HeliSimService.getCommands()
                        local engineCmdName = nil
                        if engineCmds then
                            for _, c in ipairs(engineCmds) do
                                if string.lower(c.name) == subCommand then
                                    engineCmdName = c.name
                                    break
                                end
                            end
                        end

                        if engineCmdName then
                            local argsStr = ""
                            for i = 3, #words do
                                if i > 3 then argsStr = argsStr .. " " end
                                argsStr = argsStr .. words[i]
                            end
                            doEngineCommand(engineCmdName, argsStr)
                        else
                            showError("Unknown command: " .. rawSubCommand)
                            showInfo("Type /hef help for available commands.")
                        end
                    end
                end
            end

            ISChat.instance.textEntry:setText("")
            return
        end
    end

    original_onCommandEntered(self)
end
