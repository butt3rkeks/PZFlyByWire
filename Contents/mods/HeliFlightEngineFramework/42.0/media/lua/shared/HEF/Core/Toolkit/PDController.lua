--[[
    PDController — Proportional-Derivative controller utility

    Stateless: takes error + error rate, returns correction value.
    Used for position correction, velocity tracking, altitude hold, etc.

    Supports optional tanh saturation for smooth clamping of large errors
    (prevents harsh discontinuity at max error boundary).
]]

PDController = {}

--- Compute PD correction: proportionalGain * error + derivativeGain * errorRate.
--- @param error number Current error (desired - actual)
--- @param errorRate number Rate of error change (derivative)
--- @param proportionalGain number Proportional gain
--- @param derivativeGain number Derivative gain
--- @return number Correction value
function PDController.compute(error, errorRate, proportionalGain, derivativeGain)
    return proportionalGain * error + derivativeGain * errorRate
end

--- Compute PD correction with tanh saturation on error.
--- Smoothly limits error contribution to [-maxError, +maxError] using
--- tanh curve instead of hard clamp. Prevents jerk at saturation boundary.
--- @param error number Current error
--- @param errorRate number Rate of error change
--- @param proportionalGain number Proportional gain
--- @param derivativeGain number Derivative gain
--- @param maxError number Saturation limit (error magnitude where tanh ≈ 0.76)
--- @return number Correction value
function PDController.computeSaturated(error, errorRate, proportionalGain, derivativeGain, maxError)
    local saturatedError = math.tanh(error / maxError) * maxError
    return proportionalGain * saturatedError + derivativeGain * errorRate
end

--- Compute PD correction for 2D (X, Z) with tanh saturation on error magnitude.
--- Saturates the error vector magnitude while preserving direction.
--- @param errX number Error in X
--- @param errZ number Error in Z
--- @param errRateX number Error rate in X
--- @param errRateZ number Error rate in Z
--- @param proportionalGain number Proportional gain
--- @param derivativeGain number Derivative gain
--- @param maxError number Saturation limit for error magnitude
--- @return number corrX, number corrZ
function PDController.compute2DSaturated(errX, errZ, errRateX, errRateZ, proportionalGain, derivativeGain, maxError)
    local errMag = math.sqrt(errX * errX + errZ * errZ)
    local saturatedMag = math.tanh(errMag / maxError) * maxError
    local scale = (errMag > 0.001) and (saturatedMag / errMag) or 0
    return proportionalGain * errX * scale + derivativeGain * errRateX,
           proportionalGain * errZ * scale + derivativeGain * errRateZ
end
