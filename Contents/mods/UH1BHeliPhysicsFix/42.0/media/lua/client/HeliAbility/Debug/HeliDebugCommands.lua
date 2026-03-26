--[[
    HeliDebugCommands — Runtime tuning via chat commands for helicopter physics

    Hooks ISChat.onCommandEntered to intercept /hpf commands (same pattern as
    DZChatCommands). Changes are session-only (SandboxVars writes not persisted).

    Parameter definitions (names, defaults, descriptions) come from HeliConfig.
    FBW gains and rotation settings come from HeliSimService (facade).
]]

if isServer() then return end

local ISChat = ISChat
local MOD_TAG = "[HPF]"

-------------------------------------------------------------------------------------
-- Chat message display (fake message object, same pattern as DZChatCommands)
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
    local PARAMS, PARAM_ORDER = HeliConfig.getParamDefs()
    showInfo("--- HeliPhysics Chat Commands ---")
    showInfo("/hpf show   — display all current values")
    showInfo("/hpf reset  — reset to sandbox defaults")
    showInfo("/hpf vel    — show current Bullet velocity")
    showInfo("/hpf log    — toggle periodic logging (1/sec to console)")
    showInfo("/hpf snap   — one-shot detailed state dump to chat")
    showInfo("/hpf record — toggle flight data CSV recording (per-frame)")
    showInfo("/hpf <param> <value> — set parameter:")
    for _, shorthand in ipairs(PARAM_ORDER) do
        local p = PARAMS[shorthand]
        showInfo("  " .. shorthand .. " — " .. p.desc .. " [default: " .. tostring(p.default) .. "]")
    end
    showInfo("FBW controller:")
    showInfo("  pgain     — position error P gain [default: 7.0]")
    showInfo("  dgain     — position error D gain [default: 0.3]")
    showInfo("  maxerr    — max position error (meters) [default: 10.0]")
    showInfo("  autolevel — tilt return speed (0.5=slow coast, 1.0=default, 2.0=snappy)")
    showInfo("  yawgain   — yaw correction strength [default: 0.9]")
    showInfo("  fstopgain — velocity damping strength (0.3=smooth, 0.8=snappy) [default: 0.3]")
end

commands["show"] = function(_)
    local PARAMS, PARAM_ORDER = HeliConfig.getParamDefs()
    showInfo("--- Current HeliPhysics Values ---")
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
    -- FBW gains
    showInfo(string.format("  pgain=%.2f  dgain=%.2f  maxerr=%.1f  autolevel=%.2f  yawgain=%.2f",
        HeliSimService.getPGain(),
        HeliSimService.getDGain(),
        HeliSimService.getMaxError(),
        HeliSimService.getAutoLevelMultiplier(),
        HeliSimService.getYawGain()))
    showInfo(string.format("  fstopgain=%.2f",
        HeliSimService.getFinalStopGain()))
end

commands["pgain"] = function(args)
    local val = tonumber(args[1])
    if not val then
        showError("Usage: /hpf pgain <number> (current: " ..
            string.format("%.2f", HeliSimService.getPGain()) .. ")")
        return
    end
    HeliSimService.setPGain(val)
    showSuccess("pgain = " .. string.format("%.2f", val))
end

commands["dgain"] = function(args)
    local val = tonumber(args[1])
    if not val then
        showError("Usage: /hpf dgain <number> (current: " ..
            string.format("%.2f", HeliSimService.getDGain()) .. ")")
        return
    end
    HeliSimService.setDGain(val)
    showSuccess("dgain = " .. string.format("%.2f", val))
end

commands["yawgain"] = function(args)
    local val = tonumber(args[1])
    if not val or val < 0 or val > 1 then
        showError("Usage: /hpf yawgain <0.0-1.0> (current: " ..
            string.format("%.2f", HeliSimService.getYawGain()) .. ") [0=none, 1=hard lock]")
        return
    end
    HeliSimService.setYawGain(val)
    showSuccess("yawgain = " .. string.format("%.2f", val))
end

commands["maxerr"] = function(args)
    local val = tonumber(args[1])
    if not val or val < 1 then
        showError("Usage: /hpf maxerr <number> (current: " ..
            string.format("%.1f", HeliSimService.getMaxError()) .. ") [min: 1.0]")
        return
    end
    HeliSimService.setMaxError(val)
    showSuccess("maxerr = " .. string.format("%.1f", val))
end

commands["fstopgain"] = function(args)
    local val = tonumber(args[1])
    if not val or val < 0 then
        showError("Usage: /hpf fstopgain <number> (current: " ..
            string.format("%.1f", HeliSimService.getFinalStopGain()) ..
            ") [0=disabled, higher=crisper stop]")
        return
    end
    HeliSimService.setFinalStopGain(val)
    showSuccess("fstopgain = " .. string.format("%.1f", val))
end

commands["autolevel"] = function(args)
    local val = tonumber(args[1])
    if not val or val < 0.1 then
        showError("Usage: /hpf autolevel <number> (current: " ..
            string.format("%.2f", HeliSimService.getAutoLevelMultiplier()) ..
            ") [0.5=slow coast, 1.0=default, 2.0=snappy stop]")
        return
    end
    HeliSimService.setAutoLevelMultiplier(val)
    showSuccess("autolevel = " .. string.format("%.2f", val))
end

commands["reset"] = function(_)
    HeliConfig.resetAll()
    showSuccess("All parameters reset to sandbox defaults.")
end

commands["vel"] = function(_)
    local player = getPlayer()
    local vehicle = player and player:getVehicle()
    if not vehicle then
        showError("Not in a vehicle.")
        return
    end
    local vx, vy, vz = HeliVelocityAdapter.getVelocity(vehicle)
    showValue(string.format("Bullet velocity: X=%.3f  Y(height)=%.3f  Z=%.3f", vx, vy, vz))
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
            showInfo("Fly, then /hpf record again to stop. Auto-stops on exit vehicle.")
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
    showInfo("--- HeliPhysics Snapshot ---")
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

        if #words >= 1 and string.lower(words[1]) == "/hpf" then
            local subCommand = words[2] and string.lower(words[2]) or "help"
            local handler = commands[subCommand]
            local PARAMS = HeliConfig.getParamDefs()

            if handler then
                local args = {}
                for i = 3, #words do
                    args[#args + 1] = words[i]
                end
                handler(args)
            elseif PARAMS[subCommand] then
                -- Direct parameter set: /hpf <param> <value>
                if not words[3] then
                    local val = HeliConfig.get(subCommand)
                    showValue(subCommand .. " = " .. string.format("%.2f", val))
                else
                    local value = tonumber(words[3])
                    if not value then
                        showError("Invalid number: " .. tostring(words[3]))
                    else
                        local oldVal = HeliConfig.get(subCommand)
                        HeliConfig.set(subCommand, value)
                        showSuccess(subCommand .. ": " .. string.format("%.2f", oldVal) ..
                            " -> " .. string.format("%.2f", value))
                    end
                end
            else
                showError("Unknown command: " .. subCommand)
                showInfo("Type /hpf help for available commands.")
            end

            ISChat.instance.textEntry:setText("")
            return
        end
    end

    original_onCommandEntered(self)
end
