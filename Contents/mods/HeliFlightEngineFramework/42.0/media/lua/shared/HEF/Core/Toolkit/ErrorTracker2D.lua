--[[
    ErrorTracker2D — Ring buffer position error tracker with derivative

    Records actual and desired positions each frame. Computes saturated
    error (tanh) and error rate (derivative over N frames) for PD correction.

    Instance-based: create with ErrorTracker2D.new() so multiple can coexist.
    No game API dependencies — pure math.

    NOTE: math.tanh is not available in PZ Kahlua. Tanh is computed manually
    via exp: tanh(x) = (e^2x - 1) / (e^2x + 1).
]]

ErrorTracker2D = {}
ErrorTracker2D.__index = ErrorTracker2D

--- Create a new error tracker.
--- @param historySize number Ring buffer capacity (frames, default 30)
--- @param lookbackFrames number Frames to look back for derivative (default 5)
--- @return ErrorTracker2D
function ErrorTracker2D.new(historySize, lookbackFrames)
    historySize = historySize or 30
    local self = setmetatable({
        _size = historySize,
        _lookback = lookbackFrames or 5,
        _actualX = {}, _actualZ = {},
        _desiredX = {}, _desiredZ = {},
        _idx = 0, _count = 0,
    }, ErrorTracker2D)
    for i = 1, historySize do
        self._actualX[i] = 0; self._actualZ[i] = 0
        self._desiredX[i] = 0; self._desiredZ[i] = 0
    end
    return self
end

-------------------------------------------------------------------------------------
-- Tanh saturation: smoothly limits error magnitude.
-- Small error: ~linear. Large error: asymptotes to maxError.
-------------------------------------------------------------------------------------
local function saturateError(rawX, rawZ, maxError)
    local rawMagnitude = math.sqrt(rawX * rawX + rawZ * rawZ)
    if rawMagnitude > 0.01 then
        local expDoubleScaled = math.exp(2 * rawMagnitude / maxError)
        local scale = maxError * (expDoubleScaled - 1) / ((expDoubleScaled + 1) * rawMagnitude)
        return rawX * scale, rawZ * scale
    end
    return rawX, rawZ
end

-------------------------------------------------------------------------------------
-- Public API
-------------------------------------------------------------------------------------

--- Reset history buffer for new session.
function ErrorTracker2D:reset()
    self._idx = 0
    self._count = 0
    for i = 1, self._size do
        self._actualX[i] = 0; self._actualZ[i] = 0
        self._desiredX[i] = 0; self._desiredZ[i] = 0
    end
end

--- Record one frame of actual + desired positions.
--- @param actualX number Actual position X
--- @param actualZ number Actual position Z
--- @param desiredX number Desired/simulated position X
--- @param desiredZ number Desired/simulated position Z
function ErrorTracker2D:record(actualX, actualZ, desiredX, desiredZ)
    self._idx = (self._idx % self._size) + 1
    self._actualX[self._idx] = actualX
    self._actualZ[self._idx] = actualZ
    self._desiredX[self._idx] = desiredX
    self._desiredZ[self._idx] = desiredZ
    if self._count < self._size then
        self._count = self._count + 1
    end
end

--- Compute saturated position error and error rate from history.
--- @param maxError number Tanh saturation limit (meters)
--- @return number errX, number errZ, number errRateX, number errRateZ
function ErrorTracker2D:getError(maxError)
    if self._count < 1 or self._idx < 1 then
        return 0, 0, 0, 0
    end

    local idx = self._idx
    local errX, errZ = saturateError(
        self._desiredX[idx] - self._actualX[idx],
        self._desiredZ[idx] - self._actualZ[idx],
        maxError)

    local errRateX, errRateZ = 0, 0
    local lookback = math.min(self._lookback, self._count - 1)
    if lookback >= 1 then
        local lookbackIndex = ((idx - 1 - lookback + self._size) % self._size) + 1
        local oldErrX, oldErrZ = saturateError(
            self._desiredX[lookbackIndex] - self._actualX[lookbackIndex],
            self._desiredZ[lookbackIndex] - self._actualZ[lookbackIndex],
            maxError)
        errRateX = (errX - oldErrX) / lookback
        errRateZ = (errZ - oldErrZ) / lookback
    end

    return errX, errZ, errRateX, errRateZ
end

--- Get scalar error magnitude (convenience).
--- @param maxError number Tanh saturation limit
--- @return number Error magnitude
function ErrorTracker2D:getErrorMagnitude(maxError)
    local errX, errZ = self:getError(maxError)
    return math.sqrt(errX * errX + errZ * errZ)
end

--- Clear history (used when old data becomes stale, e.g. after re-anchor).
function ErrorTracker2D:clearHistory()
    self._count = 0
end
