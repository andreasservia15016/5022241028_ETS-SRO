sim = require 'sim'

function sysCall_init()
    visionSensorHandle = sim.getObject("../sensor")
    sensorBaseHandle   = sim.getObject('..')

    maxScanDistance = 8
    sim.setObjectFloatParam(visionSensorHandle, sim.visionfloatparam_far_clipping, maxScanDistance)
    maxScanDistance_ = maxScanDistance * 0.999

    scanningAngle = 45 * math.pi / 180
    sim.setObjectFloatParam(visionSensorHandle, sim.visionfloatparam_perspective_angle, scanningAngle)

    local res = sim.getVisionSensorRes(visionSensorHandle)
    W, H = res[1], res[2]
    focal = (W / 2) / math.tan(scanningAngle / 2)

    showPoints = true
    floatingView = sim.floatingViewAdd(0.2, 0.8, 0.4, 0.4, 0)
    sim.adjustView(floatingView, visionSensorHandle, 64)

    points = sim.addDrawingObject(sim.drawing_spherepts, 0.01, 0, -1, 100000, nil, nil, nil, {1, 0, 0})
end

function sysCall_cleanup()
    if floatingView then
        sim.floatingViewRemove(floatingView)
    end
end

function sysCall_sensing()
    local measuredData = {}

    -- PENTING: jangan panggil sim.handleVisionSensor di sini.
    -- Explicit handling OFF, jadi sensor sudah diproses otomatis.
    sim.addDrawingObjectItem(points, nil)

    local buf = sim.getVisionSensorDepth(visionSensorHandle, 1)  -- 1 = depth dalam meter
    if type(buf) == 'table' then buf = table.concat(buf) end
    if type(buf) == 'string' then buf = sim.unpackFloatTable(buf) end
    if not buf or #buf < W * H then return end

    local m1  = sim.getObjectMatrix(visionSensorHandle)
    local m01 = sim.multiplyMatrices(sim.getMatrixInverse(sim.getObjectMatrix(sensorBaseHandle)), m1)

    for j = 0, H - 1 do          -- baris 0 = paling bawah
        for i = 0, W - 1 do
            local z = buf[j * W + i + 1]
            if z and z < maxScanDistance_ then
                local p = {(i + 0.5 - W / 2) / focal * z,
                           (j + 0.5 - H / 2) / focal * z,
                           z}
                local pb = sim.multiplyVector(m01, p)      -- relatif badan drone
                measuredData[#measuredData + 1] = pb[1]
                measuredData[#measuredData + 1] = pb[2]
                measuredData[#measuredData + 1] = pb[3]
                if showPoints then
                    sim.addDrawingObjectItem(points, sim.multiplyVector(m1, p))  -- koordinat dunia
                end
            end
        end
    end

    if sim.setBufferSignal then
        sim.setBufferSignal('scanData', sim.packFloatTable(measuredData))
    else
        sim.setStringSignal('scanData', sim.packFloatTable(measuredData))
    end
end
