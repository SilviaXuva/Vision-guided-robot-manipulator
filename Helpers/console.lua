jointHandles={}
deg = math.pi/180
setPos = {0,0,0,0,0,0,0}
function GetJointHandles()
    for i=1,7,1 do
        jointHandles[i]=sim.getObject('./joint'..(i-1))
    end
end
function SetJointPositions()
    for i=1,7,1 do
        sim.setJointPosition(jointHandles[i], setPos[i])
    end
end
GetJointHandles()
SetJointPositions()

function GetJointPositions()
    getPos = {}
    for i=1,7,1 do
        getPos[i] = sim.getJointPosition(jointHandles[i])
    end
    return getPos
end

tip = sim.getObject('./tip')
abg = sim.getObjectOrientation(tip, sim.handle_world)
y, p, r = sim.alphaBetaGammaToYawPitchRoll(abg[1], abg[2], abg[3])

pos = {
    -0.01647629216313362, 
    0.037338417023420334, 
    0.0009847808396443725, 
    0.07846628129482269, 
    -0.0013139393413439393, 
    0.04261644929647446, 
    0.017349982634186745
}

sim.addDrawingObjectItem(1, None)
sim.addDrawingObjectItem(2, None)
sim.removeDrawingObject(1)
sim.removeDrawingObject(2)

function RemoveDrawing() 
    for i=1,100,1 do
        sim.addDrawingObjectItem(i, nil)
        sim.addDrawingObjectItem(i, nil)
    end
end

camera=sim.getObject('./camera1')
c_pos = sim.getObjectPosition(camera, sim.handle_world)
c_abg = sim.getObjectOrientation(camera, sim.handle_world)
c_ry, c_rp, c_rr = sim.alphaBetaGammaToYawPitchRoll(c_abg[1], c_abg[2], c_abg[3])