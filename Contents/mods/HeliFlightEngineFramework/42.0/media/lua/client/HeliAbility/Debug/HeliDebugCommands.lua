--[[
    HeliDebugCommands — Runtime tuning via chat commands

    Hooks ISChat.onCommandEntered to intercept /hef commands.
    Auto-discovers tunables from active engine via HeliSimService.getTunables().
    Changes are session-only.
]]

if isServer() then return end

local ISChat = ISChat
local MOD_TAG = "[HEF]"

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
-- Command handlers
-------------------------------------------------------------------------------------

local commands = {}

commands["help"] = function(_)
    local info = HeliSimService.getInfo()
    showInfo("--- HEF Chat Commands (engine: " .. (info and info.name or "?") .. ") ---")
    showInfo("/hef show    — display all current values")
    showInfo("/hef reset   — reset tunables to defaults")
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
            showError("  WARNING: 0-wheel vehicle. Spawn a NEW helicopter for 4-wheel override.")
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
    HeliConfig.resetAll()
    -- Reset engine tunables to defaults
    local tunables = HeliSimService.getTunables()
    if tunables then
        for _, t in ipairs(tunables) do
            HeliSimService.setTunable(t.name, t.default)
        end
    end
    showSuccess("All parameters reset to defaults.")
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
    showValue(string.format("State: %s  |  Z: %.2f  |  Ground: %.2f", s.state or "?", s.curr_z or 0, s.nowMaxZ or 0))
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
    showValue(string.format("gravity=%.1f  kp=%.1f  brake=%.2f  hspeed=%.1f",
        HeliConfig.get("gravity"), HeliConfig.get("kp"), HeliConfig.get("brake"), HeliConfig.get("hspeed")))
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
            local handler = commands[subCommand]

            if handler then
                local args = {}
                for i = 3, #words do
                    args[#args + 1] = words[i]
                end
                handler(args)
            else
                -- Try as engine tunable
                local tunables = HeliSimService.getTunables()
                local tunableMatch = nil
                if tunables then
                    for _, t in ipairs(tunables) do
                        if t.name == subCommand then
                            tunableMatch = t
                            break
                        end
                    end
                end

                if tunableMatch then
                    if not words[3] then
                        showValue(subCommand .. " = " .. string.format("%.2f", tunableMatch.value))
                    else
                        local value = tonumber(words[3])
                        if not value then
                            showError("Invalid number: " .. tostring(words[3]))
                        else
                            local oldVal = tunableMatch.value
                            HeliSimService.setTunable(subCommand, value)
                            local newVal = HeliSimService.getTunable(subCommand)
                            showSuccess(subCommand .. ": " .. string.format("%.2f", oldVal) ..
                                " -> " .. string.format("%.2f", newVal))
                        end
                    end
                else
                    -- Try as HeliConfig param
                    local PARAMS = HeliConfig.getParamDefs()
                    if PARAMS[subCommand] then
                        if not words[3] then
                            local val = HeliConfig.get(subCommand)
                            showValue(subCommand .. " = " .. string.format("%.2f", val))
                        else
                            local value = tonumber(words[3])
                            if not value then
                                showError("Invalid number: " .. tostring(words[3]))
                            else
                                local oldVal = HeliConfig.get(subCommand)
                                local clamped = HeliConfig.set(subCommand, value)
                                showSuccess(subCommand .. ": " .. string.format("%.2f", oldVal) ..
                                    " -> " .. string.format("%.2f", clamped))
                            end
                        end
                    else
                        -- Try as engine command
                        local engineCmds = HeliSimService.getCommands()
                        local isEngineCmd = false
                        if engineCmds then
                            for _, c in ipairs(engineCmds) do
                                if c.name == subCommand then
                                    isEngineCmd = true
                                    break
                                end
                            end
                        end

                        if isEngineCmd then
                            local argsStr = ""
                            for i = 3, #words do
                                if i > 3 then argsStr = argsStr .. " " end
                                argsStr = argsStr .. words[i]
                            end
                            local response = HeliSimService.executeCommand(subCommand, argsStr)
                            if response then showInfo(response) end
                        else
                            showError("Unknown command: " .. subCommand)
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
