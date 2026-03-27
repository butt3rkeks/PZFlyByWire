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

-------------------------------------------------------------------------------------
-- Ghost mode occupant tracking
--
-- Tracks all players we've put into ghost mode so we can reliably clean them up
-- even if they leave the vehicle unexpectedly (crash, disconnect, seat switch,
-- kicked). Each frame we reconcile: current occupants get ghost mode applied,
-- tracked players no longer in the vehicle get cleared.
-------------------------------------------------------------------------------------
local _ghostedPlayers = {}  -- { [IsoGameCharacter] = true }

--- Clear ghost mode + Z on a single character and remove from tracking.
--- @param character IsoGameCharacter
local function clearOccupantFlightState(character)
    if character:isGhostMode() and not character:isGodMod() then
        character:setGhostMode(false)
    end
    character:setZ(0)
    _ghostedPlayers[character] = nil
end

--- Apply ghost mode + Z-level to a single character and add to tracking.
--- @param character IsoGameCharacter
--- @param altitude number Current altitude (continuous Z from Bullet physics)
local function setOccupantFlightState(character, altitude)
    if not character:isGhostMode() then
        character:setGhostMode(true)
    end
    character:setZ(altitude)
    _ghostedPlayers[character] = true
end

--- Update ghost mode and Z-level for all occupants based on altitude.
--- Builds a set of current occupants, applies flight state to each.
--- Then sweeps tracked players — anyone no longer in the vehicle gets cleared.
--- Handles: normal exit, crash/disconnect, seat switch to non-heli vehicle.
--- @param playerObj IsoPlayer The pilot (seat 0)
--- @param currentAltitude number Current height in PZ Z-levels
--- @param vehicle BaseVehicle The helicopter
function HeliAuxiliary.updateGhostMode(playerObj, currentAltitude, vehicle)
    local airborne = currentAltitude > 1

    -- Build set of current occupants
    local currentOccupants = {}
    local seatCount = vehicle:getScript():getPassengerCount()
    for seat = 0, seatCount - 1 do
        local occupant = vehicle:getCharacter(seat)
        if occupant then
            currentOccupants[occupant] = true
            if airborne then
                setOccupantFlightState(occupant, currentAltitude)
            else
                clearOccupantFlightState(occupant)
            end
        end
    end

    -- Sweep: clear anyone we ghosted who is no longer in the vehicle
    for player, _ in pairs(_ghostedPlayers) do
        if not currentOccupants[player] then
            clearOccupantFlightState(player)
        end
    end
end

--- Update the NightLight system.
--- Creates/moves a light source at ground level below the helicopter.
--- @param playerObj IsoPlayer
--- @param vehicle BaseVehicle
--- @param groundLevelZ number Ground level Z (from HeliTerrainUtil.getNowMaxZ)
function HeliAuxiliary.updateNightLight(playerObj, vehicle, groundLevelZ)
    local modData = vehicle:getModData()
    if not modData.NightLight then
        modData.NightLight = false
    end
    if modData.NightLight then
        -- Use integer floor Z for light placement — PZ lighting only illuminates
        -- tiles on the same integer Z-level. groundLevelZ is fractional (e.g. 0.4).
        local lightZ = math.floor(groundLevelZ)
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
--- Airborne (z>1): 0.03 * fpsMultiplier * gasConsumptionRate per tick.
--- Grounded (z<=1): 0.01 * fpsMultiplier * gasConsumptionRate per tick.
--- @param vehicle BaseVehicle
--- @param currentAltitude number Current height
--- @param fpsMultiplier number Frame-rate compensation factor
--- @param heliType string HeliList key (e.g. "UH1BHuey")
function HeliAuxiliary.consumeGas(vehicle, currentAltitude, fpsMultiplier, heliType)
    local gasTankType = "GasTank"
    local gasConsumptionRate = HeliList[heliType].GasMultiples * (SandboxVars.WT and SandboxVars.WT.HeliGasMultiples or 1) * 0.1
    local gasTank = vehicle:getPartById(gasTankType)
    if gasTank and gasTank:getInventoryItem() then
        local rate
        if currentAltitude > 1 then
            rate = 0.03 * fpsMultiplier * gasConsumptionRate
        else
            rate = 0.01 * fpsMultiplier * gasConsumptionRate
        end
        gasTank:setContainerContentAmount(math.max(gasTank:getContainerContentAmount() - rate, 0))
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
            local currentCondition = part:getCondition()
            if ZombRand(1, 100) <= 60 then
                Panzer:sendVehicleCommandWrapper(getPlayer(), part, "setPartCondition", {
                    condition = currentCondition - ZombRand(4, 10)
                })
            end
        end
    end
end

--- Cleanup when pilot exits or flight ends.
--- Clears ghost mode + Z for all tracked occupants.
--- @param player IsoPlayer The pilot exiting (also cleared directly as safety net)
function HeliAuxiliary.cleanup(player)
    -- Clear all tracked occupants (includes pilot if tracked)
    for character, _ in pairs(_ghostedPlayers) do
        clearOccupantFlightState(character)
    end
    -- Safety net: ensure pilot is cleared even if not in tracking table
    if player:isGhostMode() and not player:isGodMod() then
        player:setGhostMode(false)
    end
    player:setZ(0)
end
