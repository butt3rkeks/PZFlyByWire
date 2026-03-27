--[[
    HeliTerrainUtil — World geometry queries for helicopter flight

    Provides terrain and collision detection:
    - getNowMaxZ: scans downward for solid floor (landing surface detection)
    - isBlocked: scans forward for walls (collision avoidance)

    Pure game-world reads — no vehicle physics, no input handling.
    Uses :has() (B42.15+ API, renamed from :Is()).
]]

HeliTerrainUtil = {}

HeliTerrainUtil.BanFlagList = {
    IsoFlagType.WallW,
    IsoFlagType.WallN,
    IsoFlagType.WallSE,
    IsoFlagType.WallNW,
}

--- Get objects on a grid square.
--- @param sq IsoGridSquare
--- @return IsoGridSquare|false, table|nil objects, table|nil objectArray
local function getSquareObjects(sq)
    if sq == nil then
        return false
    end
    local squareObjects = sq:getObjects()
    local objectCount = squareObjects:size()
    local objectList = {}
    for i = objectCount - 1, 0, -1 do
        local obj = squareObjects:get(i)
        table.insert(objectList, obj)
    end
    return sq, squareObjects, objectList
end

--- Check if a grid square has any of the given IsoFlagType flags.
--- @param square IsoGridSquare
--- @param flagTable table Array of IsoFlagType values
--- @return boolean
local function hasFlag(square, flagTable)
    local sq, squareObjects, objectList = getSquareObjects(square)
    if objectList then
        for i = 1, #objectList do
            local obj = objectList[i]
            local sprite = obj:getSprite()
            if sprite and sprite:getProperties() then
                for j = 1, #flagTable do
                    if sprite:getProperties():has(flagTable[j]) then
                        return true
                    end
                end
            end
        end
    end
    return false
end

-- Expose for use by other modules
HeliTerrainUtil.hasFlag = hasFlag

--- Scan downward from current Z to find the nearest solid floor.
--- Returns the Z-level + 0.4 (soft landing margin).
--- @param playerObj IsoPlayer
--- @param currentAltitude number Current helicopter height (PZ Z-levels)
--- @return number Landing height (Z-level + 0.4)
function HeliTerrainUtil.getNowMaxZ(playerObj, currentAltitude)
    local maxZ = 0
    local zNow = math.floor(currentAltitude)
    while true do
        local sq = getCell():getGridSquare(playerObj:getX(), playerObj:getY(), zNow)
        if hasFlag(sq, { IsoFlagType.solidfloor }) then
            maxZ = zNow
            break
        else
            zNow = zNow - 1
            if zNow <= 0 then
                maxZ = zNow
                break
            end
        end
    end
    return maxZ + 0.4
end

-------------------------------------------------------------------------------------
-- Per-frame cache for isBlocked results.
-- isBlocked scans 132 grid squares per call. Both HeliInputProcessor (tilt reversal)
-- and HeliSimService (thrust blocking) call it with the same direction in the same
-- frame. The cache eliminates the redundant scan.
-- HeliMove calls invalidateBlockedCache() once per tick before any isBlocked calls.
-------------------------------------------------------------------------------------
local _blockedCache = {}
local _blockedCacheValid = false

function HeliTerrainUtil.invalidateBlockedCache()
    _blockedCacheValid = false
end

--- Check if the helicopter's path in a given direction is blocked by walls.
--- Scans a grid of squares ahead based on the player's heading and direction offset.
--- Results are cached per frame — call invalidateBlockedCache() each tick.
--- @param playerObj IsoPlayer
--- @param direction string "UP", "DOWN", "LEFT", or "RIGHT"
--- @param vehicle BaseVehicle
--- @param scratchVector Vector3f Reusable temp vector
--- @return boolean True if blocked by walls
function HeliTerrainUtil.isBlocked(playerObj, direction, vehicle, scratchVector)
    -- Return cached result if available this frame
    if _blockedCacheValid and _blockedCache[direction] ~= nil then
        return _blockedCache[direction]
    end
    if not _blockedCacheValid then
        _blockedCache = {}
        _blockedCacheValid = true
    end
    local scannedSquares = {}
    local playerAngle = playerObj:getForwardDirection():getDirection()

    local vehicleX = vehicle:getX()
    local vehicleY = vehicle:getY()
    local vehicleZ = math.floor(vehicle:getWorldPos(0, 0, 0, scratchVector):z())
    local scanMinForward, scanMaxForward, scanMinLateral, scanMaxLateral = 0, 10, 0, 5

    if direction == "DOWN" then
        playerAngle = playerAngle + 60
    elseif direction == "LEFT" then
        playerAngle = playerAngle - 90
    elseif direction == "RIGHT" then
        playerAngle = playerAngle + 90
    end

    for z = scanMinForward, scanMaxForward do
        for i = scanMinLateral, scanMaxLateral do
            local probeX1 = vehicleX + z / 2 * math.cos(playerAngle) + i / 3 * math.cos(playerAngle + math.pi / 2)
            local probeY1 = vehicleY + z / 2 * math.sin(playerAngle) + i / 3 * math.sin(playerAngle + math.pi / 2)
            local sq1 = getCell():getGridSquare(probeX1, probeY1, vehicleZ)
            if sq1 then scannedSquares[sq1] = 1 end
            local probeX2 = vehicleX + z / 2 * math.cos(playerAngle) + i / 3 * math.cos(playerAngle - math.pi / 2)
            local probeY2 = vehicleY + z / 2 * math.sin(playerAngle) + i / 3 * math.sin(playerAngle - math.pi / 2)
            local sq2 = getCell():getGridSquare(probeX2, probeY2, vehicleZ)
            if sq2 then scannedSquares[sq2] = 1 end
        end
    end

    for k, _ in pairs(scannedSquares) do
        if k then
            if hasFlag(k, HeliTerrainUtil.BanFlagList) then
                _blockedCache[direction] = true
                return true
            end
        end
    end
    _blockedCache[direction] = false
    return false
end
