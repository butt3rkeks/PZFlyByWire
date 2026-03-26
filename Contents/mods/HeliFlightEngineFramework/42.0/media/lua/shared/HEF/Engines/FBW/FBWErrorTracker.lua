--[[
    FBWErrorTracker — History buffer + tanh error saturation

    Owns the position history ring buffer and computes saturated position error
    + error rate for PD correction. Pure math: given (simPos, actualPos) →
    saturated error + rate.

    No Bullet, no flags, no gains. Caller provides positions each frame.
]]

FBWErrorTracker = {}

-------------------------------------------------------------------------------------
-- History ring buffer
-------------------------------------------------------------------------------------
local _historySize = 30  -- read from HeliConfig at init time
local _historyX = {}
local _historyZ = {}
local _historySimX = {}
local _historySimZ = {}
local _historyIdx = 0
local _historyCount = 0

-- Pre-fill arrays
for _i = 1, _historySize do
    _historyX[_i] = 0
    _historyZ[_i] = 0
    _historySimX[_i] = 0
    _historySimZ[_i] = 0
end

-------------------------------------------------------------------------------------
-- Soft saturation via tanh: smoothly limits error magnitude.
-- effective = maxerr * tanh(raw / maxerr)
-- Small error: ~linear. Large error: asymptotes to maxerr.
-- math.tanh not available in Kahlua; computed manually.
-------------------------------------------------------------------------------------
local function saturateError(rawX, rawZ, maxError)
    local rawMag = math.sqrt(rawX * rawX + rawZ * rawZ)
    if rawMag > 0.01 then
        local e2x = math.exp(2 * rawMag / maxError)
        local scale = maxError * (e2x - 1) / ((e2x + 1) * rawMag)
        return rawX * scale, rawZ * scale
    end
    return rawX, rawZ
end

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset history buffer for new flight session.
function FBWErrorTracker.reset()
    _historySize = HeliConfig.HISTORY_SIZE
    _historyIdx = 0
    _historyCount = 0
    for i = 1, _historySize do
        _historyX[i] = 0
        _historyZ[i] = 0
        _historySimX[i] = 0
        _historySimZ[i] = 0
    end
end

--- Record one frame of actual + simulated positions into the ring buffer.
--- @param actualX number Actual vehicle X position
--- @param actualZ number Actual vehicle Z position
--- @param simPosX number Simulated ideal X position
--- @param simPosZ number Simulated ideal Z position
function FBWErrorTracker.record(actualX, actualZ, simPosX, simPosZ)
    _historyIdx = (_historyIdx % _historySize) + 1
    _historyX[_historyIdx] = actualX
    _historyZ[_historyIdx] = actualZ
    _historySimX[_historyIdx] = simPosX
    _historySimZ[_historyIdx] = simPosZ
    if _historyCount < _historySize then
        _historyCount = _historyCount + 1
    end
end

--- Compute saturated position error and error rate from history.
--- @return number errorX, number errorZ, number errorRateX, number errorRateZ
function FBWErrorTracker.getPositionError()
    if _historyCount < 1 or _historyIdx < 1 then
        return 0, 0, 0, 0
    end

    local maxError = HeliConfig.get("maxerr")
    local currentIdx = _historyIdx
    local errorX, errorZ = saturateError(
        _historySimX[currentIdx] - _historyX[currentIdx],
        _historySimZ[currentIdx] - _historyZ[currentIdx],
        maxError)

    -- Error rate: change in SATURATED error over last N frames
    local errorRateX, errorRateZ = 0, 0
    if _historyCount >= 5 then
        local lookback = math.min(5, _historyCount - 1)
        local oldIdx = ((currentIdx - 1 - lookback + _historySize) % _historySize) + 1
        local oldErrX, oldErrZ = saturateError(
            _historySimX[oldIdx] - _historyX[oldIdx],
            _historySimZ[oldIdx] - _historyZ[oldIdx],
            maxError)

        errorRateX = (errorX - oldErrX) / lookback
        errorRateZ = (errorZ - oldErrZ) / lookback
    end

    return errorX, errorZ, errorRateX, errorRateZ
end

--- Get scalar error magnitude (convenience, avoids sqrt in caller).
--- @return number Error magnitude
function FBWErrorTracker.getErrorMagnitude()
    local errX, errZ = FBWErrorTracker.getPositionError()
    return math.sqrt(errX * errX + errZ * errZ)
end

--- Clear history (used after heading re-anchor when old data is stale).
function FBWErrorTracker.clearHistory()
    _historyCount = 0
end
