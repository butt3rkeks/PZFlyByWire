--[[
    HeliAuxiliary — Gameplay side-effects for helicopter flight

    Manages non-physics helicopter systems:
    - Ghost mode (collision immunity when airborne)
    - NightLight (dynamic light source following helicopter)
    - Gas consumption (fuel drain per tick)
    - Wall damage (part condition reduction on wall collision)
    - Cleanup on exit vehicle
]]

HeliAuxiliary = {}

-- NightLight state (per-session, not persisted)
local _lastLightSquare = nil
local _light = nil

--- Toggle ghost mode based on altitude.
--- Above z=1: enable ghost mode (no zombie/ground collision).
--- At/below z=1: disable ghost mode.
--- @param playerObj IsoPlayer
--- @param curr_z number Current height in PZ Z-levels
function HeliAuxiliary.updateGhostMode(playerObj, curr_z)
    if curr_z > 1 then
        if not playerObj:isGhostMode() then
            playerObj:setGhostMode(true)
        end
        playerObj:setZ(curr_z)
    else
        if playerObj:isGhostMode() then
            playerObj:setGhostMode(false)
        end
        playerObj:setZ(0)
    end
end

--- Update the NightLight system.
--- Creates/moves a light source at ground level below the helicopter.
--- @param playerObj IsoPlayer
--- @param vehicle BaseVehicle
--- @param nowMaxZ number Ground level Z (from HeliTerrainUtil.getNowMaxZ)
function HeliAuxiliary.updateNightLight(playerObj, vehicle, nowMaxZ)
    local modData = vehicle:getModData()
    if not modData.NightLight then
        modData.NightLight = false
    end
    if modData.NightLight then
        -- Use integer floor Z for light placement — PZ lighting only illuminates
        -- tiles on the same integer Z-level. nowMaxZ is fractional (e.g. 0.4).
        local lightZ = math.floor(nowMaxZ)
        local currentSquare = getCell():getGridSquare(playerObj:getX(), playerObj:getY(), lightZ)
        if _lastLightSquare ~= currentSquare then
            _lastLightSquare = currentSquare
            if _light then
                getCell():removeLamppost(_light)
            end
            _light = IsoLightSource.new(playerObj:getX(), playerObj:getY(), lightZ, 0.8, 0.6, 0.8, 20)
            getCell():addLamppost(_light)
        end
    else
        if _light then
            getCell():removeLamppost(_light)
            _light = nil
        end
    end
end

--- Consume fuel based on flight state.
--- Airborne (z>1): 0.03 * fpsMultiplier * gasMultiples per tick.
--- Grounded (z<=1): 0.01 * fpsMultiplier * gasMultiples per tick.
--- @param vehicle BaseVehicle
--- @param curr_z number Current height
--- @param fpsMultiplier number Frame-rate compensation factor
--- @param heliType string HeliList key (e.g. "UH1BHuey")
function HeliAuxiliary.consumeGas(vehicle, curr_z, fpsMultiplier, heliType)
    local gasTankType = "GasTank"
    local gasMultiples = HeliList[heliType].GasMultiples * (SandboxVars.WT and SandboxVars.WT.HeliGasMultiples or 1) * 0.1
    local gasTank = vehicle:getPartById(gasTankType)
    if gasTank and gasTank:getInventoryItem() then
        local rate
        if curr_z > 1 then
            rate = 0.03 * fpsMultiplier * gasMultiples
        else
            rate = 0.01 * fpsMultiplier * gasMultiples
        end
        gasTank:setContainerContentAmount(gasTank:getContainerContentAmount() - rate)
    end
end

--- Apply damage to helicopter parts when colliding with walls.
--- Reduces random parts' condition by 4-10 points with 60% chance.
--- Uses Panzer network command for MP sync.
--- @param vehicle BaseVehicle
function HeliAuxiliary.applyWallDamage(vehicle)
    vehicle:playSound("JetAlarm")
    if SandboxVars.WT and SandboxVars.WT.FloorHit then
        for i = 1, vehicle:getPartCount() do
            local part = vehicle:getPartByIndex(i - 1)
            local condNow = part:getCondition()
            if ZombRand(1, 100) <= 60 then
                Panzer:sendVehicleCommandWrapper(getPlayer(), part, "setPartCondition", {
                    condition = condNow - ZombRand(4, 10)
                })
            end
        end
    end
end

--- Cleanup when player exits helicopter.
--- Disables ghost mode if it was enabled during flight.
--- @param player IsoPlayer
function HeliAuxiliary.cleanup(player)
    if player:isGhostMode() and not player:isGodMod() then
        player:setGhostMode(false)
    end
end
