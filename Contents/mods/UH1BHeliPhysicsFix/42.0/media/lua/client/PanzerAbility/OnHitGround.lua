local function mathfloor(number)
    return number - math.floor(number)
end
local function itemremove(worlditem)
    if worlditem == nil then
        return
    end
    -- worlditem:getWorldItem():getSquare():transmitRemoveItemFromSquare(worlditem:getWorldItem())
    worlditem:getWorldItem():removeFromSquare()
end

local function additemsfx(square, itemname, x, y, z)
    if square:getZ() > 7 then
        return
    end
    local itemin = square:AddWorldInventoryItem(itemname, x, y, z)
    -- itemin:setWorldItem(false)
    local chunk = square:getChunk()
    if chunk then
        square:getChunk():recalcHashCodeObjects()
    else
        return
    end
    return itemin
end

local boomtable = {}

local function boomontick()
    local tablenow = boomtable
    for kt, vt in pairs(tablenow) do
        for kz, vz in pairs(vt[12]) do
            -- itemremove(vt[12][vt[3] - vt[13]])
        end
        if vt[3] > vt[2] + vt[13] then
            tablenow[kt] = nil
            break
        end

        if vt[3] == 1 and vt[7] == 0 then
            local itemornone = additemsfx(vt[5], vt[1] .. tostring(vt[3]), vt[4][1], vt[4][2], vt[4][3])
            table.insert(vt[12], itemornone)
            vt[3] = vt[3] + 1
        elseif vt[7] > vt[6] and vt[3] <= vt[2] then
            vt[7] = 0
            local itemornone = additemsfx(vt[5], vt[1] .. tostring(vt[3]), vt[4][1], vt[4][2], vt[4][3])
            table.insert(vt[12], itemornone)
            vt[3] = vt[3] + 1
        elseif vt[7] > vt[6] then
            vt[7] = 0
            vt[3] = vt[3] + 1
        end
        vt[7] = vt[7] + 1
    end
end

local function boomsfx(sq, sfxName, sfxNum, ticktime)
    if not sq then
        sq = getPlayer():getCurrentSquare()
    end
    local sfxname = sfxName or "Base.theTigerSFX"
    local sfxnum = sfxNum or 12
    local nowsfxnum = 1
    local sfxcount = 0
    local pos = { sq:getX(), sq:getY(), sq:getZ() }
    local square = sq
    local ticktime = ticktime or 1
    local func = function()
        return
    end
    local varz1, varz2, varz3
    local item = {}
    local offset = 1

    local tablesfx = { sfxname, ---1
        sfxnum,                 ---2
        nowsfxnum,              ---3
        pos,                    ---4
        square,                 ---5
        ticktime,               ---6
        sfxcount,               ---7
        func,                   ---8
        varz1,                  ---9
        varz2,                  ---10
        varz3,                  ---11
        item,                   ---12
        offset                  ---13滞后
    }

    table.insert(boomtable, tablesfx)
end

local function sqobject(sqa)
    local cell = getWorld():getCell();
    local sq = sqa
    if sq == nil then
        return false;
    end
    local sqObjs = sq:getObjects();
    local sqSize = sqObjs:size();
    local tbl = {}
    for i = sqSize - 1, 0, -1 do
        local obj = sqObjs:get(i);
        table.insert(tbl, obj)
    end
    return sq, sqObjs, tbl, cell
end

local function KillReachableSquaresZombie(square, range, Damage, toHouse, Vehicle, ash)
    if not square then
        return
    end
    if range == nil then
        range = 1
    end
    local cx = square:getX()
    local cy = square:getY()
    local cz = square:getZ()
    local dx, dy = 0, 0
    local currentSq = square
    Damage = Damage + Damage * (GetAbilityLevel(getPlayer():getVehicle(), "Aim") / 10)
    currentSq:playSound("30mmBoom");
    for dy = 0 - range, range do
        for dx = 0 - range, range do
            local Cell = getCell()
            local square = Cell:getGridSquare(cx + dx, cy + dy, cz)
            if square ~= currentSq and currentSq and currentSq:isBlockedTo(square) then
                square = nil -- 不读取被隔开的区块
            end
            if square then
                if SandboxVars.WT.DestoryStartAsh then
                    if ash then
                        if ZombRand(30) <= 4 then
                            local objectz = IsoObject.new(square, "floors_burnt_01_0", "", false)
                            square:AddTileObject(objectz)
                        elseif ZombRand(30) <= 6 then
                            local objectz = IsoObject.new(square, "floors_burnt_01_1", "", false)
                            square:AddTileObject(objectz)
                        elseif ZombRand(25) <= 6 then
                            local objectz = IsoObject.new(square, "floors_burnt_01_2", "", false)
                            square:AddTileObject(objectz)
                        end
                    end
                end
                local Zombies = square:getMovingObjects()
                for i = 0, Zombies:size() - 1 do
                    local zombie = Zombies:get(i)
                    if instanceof(zombie, "IsoZombie") then
                        local distance = math.sqrt((dx * dx) + (dy * dy))
                        local scaledDamage = Damage * (math.abs((range - distance)) / range) * 2
                        local newHealth = zombie:getHealth() - scaledDamage
                        if newHealth < 0 then
                            newHealth = 0
                            zombie:Kill(getPlayer())
                            local modData = Vehicle:getModData()
                            if not modData.researchPoint then
                                modData.researchPoint = 0
                            end
                            if not modData.CrewKillNum then
                                modData.CrewKillNum = 0
                            end
                            modData.researchPoint = modData.researchPoint + 5
                            modData.CrewKillNum = modData.CrewKillNum + 5
                            if modData.CrewKillNum >= 100 then
                                modData.CrewKillNum = modData.CrewKillNum - 100
                                if not modData.CrewLevel then
                                    modData.CrewLevel = 0
                                end
                                if not modData.CrewPoint then
                                    modData.CrewPoint = 0
                                end
                                modData.CrewPoint = modData.CrewPoint + 1
                                modData.CrewLevel = modData.CrewLevel + 1
                            end
                        end
                        zombie:setHealth(newHealth)
                    end
                end
                if SandboxVars.WT.TigerDestoryNormalCar then
                    local vehicle = square:getVehicleContainer()
                    if vehicle and not GetPanzerType(vehicle) then
                        if SandboxVars.WT.TigerDestoryBurnedCarOnly then
                            if string.find(vehicle:getScriptName(), "burnt") or
                                string.find(vehicle:getScriptName(), "smashed") then
                                sendClientCommand(getPlayer(), "vehicle", "remove", {
                                    vehicle = vehicle:getId()
                                })
                            end
                        else
                            sendClientCommand(getPlayer(), "vehicle", "remove", {
                                vehicle = vehicle:getId()
                            })
                        end
                    end
                end

                if SandboxVars.WT.TigerDestoryBuilding then
                    local NewSuqare = Cell:getGridSquare(cx + dx, cy + dy, cz)
                    local sq, sqObjs, objTbl, cell = sqobject(NewSuqare)
                    local safehouse = SafeHouse.isSafeHouse(NewSuqare, nil, true)
                    if safehouse then
                        return
                    end
                    if objTbl and toHouse then
                        for i = 1, #objTbl do
                            -- if SandboxVars.DestoryHouse == false or
                            --     (SafeHouse.isSafeHouse(square, nil, true) and SandboxVars.DestorySafeHouse == false) then
                            --     break
                            -- end
                            local obj = objTbl[i]
                            local sprite = obj:getSprite()
                            if sprite and sprite:getProperties():has(IsoFlagType.solidfloor) ~= true then
                                local stairObjects = buildUtil.getStairObjects(obj)
                                if #stairObjects > 0 then
                                    for i = 1, #stairObjects do
                                        if isClient() then
                                            sledgeDestroy(stairObjects[i])
                                        else
                                            stairObjects[i]:getSquare():RemoveTileObject(stairObjects[i])
                                        end
                                    end
                                else
                                    if isClient() then
                                        sledgeDestroy(obj)
                                    else
                                        sq:RemoveTileObject(obj);
                                        sq:getSpecialObjects():remove(obj);
                                        sq:getObjects():remove(obj);
                                    end
                                end
                            end
                        end
                        --------------------------------------------------------------------
                    end
                end
            end
        end
    end
end

local function MGKillReachableSquaresZombie(square, vehicle, type)
    local range = 2

    local cx = square:getX()
    local cy = square:getY()
    local cz = square:getZ()
    local dx, dy = 0, 0
    local currentSq = square
    local zombie = nil
    for dy = 0 - range, range do
        for dx = 0 - range, range do
            local square = getCell():getGridSquare(cx + dx, cy + dy, cz)

            if square then
                local Zombies = square:getMovingObjects()
                if Zombies:size() - 1 >= 0 then
                    local Tempzombie = Zombies:get(0)

                    if instanceof(Tempzombie, "IsoZombie") then
                        if Tempzombie:getHealth() > 0 then
                            zombie = Tempzombie
                            if type ~= "DoneOnly" then
                                break
                            else
                                if not Tempzombie:isCrawling() then
                                    break
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    if zombie then
        zombie:setOutlineHighlight(true)
        -- zombie:setOutlineHighlightCol(0.51, 0.51, 1)
        if type == "DoneOnly" then
            if not zombie:isCrawling() then
                zombie:toggleCrawling()
                zombie:setCanWalk(false);
                zombie:setFallOnFront(true)
            end
            return
        end
        local newHealth = zombie:getHealth() - 0.5 - 0.5 * (GetAbilityLevel(getPlayer():getVehicle(), "Aim") / 10)
        if newHealth < 0 then
            newHealth = 0
            zombie:Kill(getPlayer())
            local modData = vehicle:getModData()
            if not modData.researchPoint then
                modData.researchPoint = 0
            end
            if not modData.CrewKillNum then
                modData.CrewKillNum = 0
            end
            modData.researchPoint = modData.researchPoint + 10
            modData.CrewKillNum = modData.CrewKillNum + 10
            if modData.CrewKillNum >= 100 then
                modData.CrewKillNum = modData.CrewKillNum - 100
                if not modData.CrewLevel then
                    modData.CrewLevel = 0
                end
                if not modData.CrewPoint then
                    modData.CrewPoint = 0
                end
                modData.CrewPoint = modData.CrewPoint + 1
                modData.CrewLevel = modData.CrewLevel + 1
            end
        end
        zombie:setHealth(newHealth)
    end
end

Events.OnPlayerUpdate.Add(boomontick)

function PanzerDetectTarget(square, type)
    local playerObj = getPlayer()
    local Vehicle = playerObj:getVehicle()
    if type == "Art" then
        KillReachableSquaresZombie(square, 3, 3, true, Vehicle, true)
        boomsfx(square)
        return
    elseif type == "Air" then
        KillReachableSquaresZombie(square, 2, 2, false, Vehicle, false)
        return
    elseif type == "AnTiPersonnel" then
        KillReachableSquaresZombie(square, 5, 5, false, Vehicle, false)
        return
    elseif type == "Rocket" or type == "RocketNest" then
        KillReachableSquaresZombie(square, 2, 2, true, Vehicle, true)
        boomsfx(square)
        return
    elseif type == "AnTiTankMissile" then
        KillReachableSquaresZombie(square, 8, 8, true, Vehicle, true)
        boomsfx(square)
        return
    elseif type == "AP" then
        KillReachableSquaresZombie(square, 3, 3, false, Vehicle, true)
        boomsfx(square)
        return
    elseif type == "HE" then
        KillReachableSquaresZombie(square, 5, 2, false, Vehicle, true)
        boomsfx(square)
        return
    end
    if not Vehicle then
        return
    end
    local modData = Vehicle:getModData()
    local PanzerType = GetPanzerType(Vehicle)
    if (Vehicle and PanzerType) and Vehicle:getSeat(playerObj) == 0 then
        if type and type == "MG" then
            MGKillReachableSquaresZombie(square, Vehicle)
        else
            if modData.PanzerLoadNow == "HE" then
                KillReachableSquaresZombie(square, 5, 2, false, Vehicle, true)
            elseif modData.PanzerLoadNow == "AP" then
                KillReachableSquaresZombie(square, 3, 3, true, Vehicle, true)
            end
            -- modData.PanzerLoadNow = nil
            square:playSound("ground_explosion", true)
            boomsfx(square)
        end
    elseif (Vehicle) and Vehicle:getSeat(playerObj) == 2 then
        if GetPanzerType(Vehicle) == "99A" then
            MGKillReachableSquaresZombie(square, Vehicle, "DoneOnly")
        else
            MGKillReachableSquaresZombie(square, Vehicle)
        end
    end
    local HeliType = GetHeliType(Vehicle)
    if (Vehicle and HeliType) and Vehicle:getSeat(playerObj) == 0 then
        if modData.AmmoTypeNow == "HE" then
            KillReachableSquaresZombie(square, 5, 2, false, Vehicle, true)
        elseif modData.AmmoTypeNow == "AP" then
            KillReachableSquaresZombie(square, 3, 3, true, Vehicle, true)
        end
        square:playSound("ground_explosion", true)
        boomsfx(square)
    end
end
