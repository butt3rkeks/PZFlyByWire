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
local function sqobject(sq)
    if sq == nil then
        return false
    end
    local sqObjs = sq:getObjects()
    local sqSize = sqObjs:size()
    local tbl = {}
    for i = sqSize - 1, 0, -1 do
        local obj = sqObjs:get(i)
        table.insert(tbl, obj)
    end
    return sq, sqObjs, tbl
end

--- Check if a grid square has any of the given IsoFlagType flags.
--- @param square IsoGridSquare
--- @param flagTable table Array of IsoFlagType values
--- @return boolean
local function hasFlag(square, flagTable)
    local sq, sqObjs, objTbl = sqobject(square)
    if objTbl then
        for i = 1, #objTbl do
            local obj = objTbl[i]
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
--- @param curr_z number Current helicopter height (PZ Z-levels)
--- @return number Landing height (Z-level + 0.4)
function HeliTerrainUtil.getNowMaxZ(playerObj, curr_z)
    local maxZ = 0
    local zNow = math.floor(curr_z)
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
--- @param tempVector2 Vector3f Reusable temp vector
--- @return boolean True if blocked by walls
function HeliTerrainUtil.isBlocked(playerObj, direction, vehicle, tempVector2)
    -- Return cached result if available this frame
    if _blockedCacheValid and _blockedCache[direction] ~= nil then
        return _blockedCache[direction]
    end
    if not _blockedCacheValid then
        _blockedCache = {}
        _blockedCacheValid = true
    end
    local squaretable = {}
    local playerangle = playerObj:getForwardDirection():getDirection()

    local vx = vehicle:getX()
    local vy = vehicle:getY()
    local vz = math.floor(vehicle:getWorldPos(0, 0, 0, tempVector2):z())
    local rangeX1, rangeX2, rangeY1, rangeY2 = 0, 10, 0, 5

    if direction == "DOWN" then
        playerangle = playerangle + 60
    elseif direction == "LEFT" then
        playerangle = playerangle - 90
    elseif direction == "RIGHT" then
        playerangle = playerangle + 90
    end

    for z = rangeX1, rangeX2 do
        for i = rangeY1, rangeY2 do
            local vx1 = vx + z / 2 * math.cos(playerangle) + i / 3 * math.cos(playerangle + math.pi / 2)
            local vy1 = vy + z / 2 * math.sin(playerangle) + i / 3 * math.sin(playerangle + math.pi / 2)
            local sq1 = getCell():getGridSquare(vx1, vy1, vz)
            if sq1 then squaretable[sq1] = 1 end
            local vx2 = vx + z / 2 * math.cos(playerangle) + i / 3 * math.cos(playerangle - math.pi / 2)
            local vy2 = vy + z / 2 * math.sin(playerangle) + i / 3 * math.sin(playerangle - math.pi / 2)
            local sq2 = getCell():getGridSquare(vx2, vy2, vz)
            if sq2 then squaretable[sq2] = 1 end
        end
    end

    for k, _ in pairs(squaretable) do
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
