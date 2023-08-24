class RobotiqGripper:
    def __init__(self, client, sim):
        self.client = client
        self.sim = sim
        self.simIK = self.client.getObject('simIK')

        self.j1 = self.sim.getObject('./active1')
        self.j2 = self.sim.getObject('./active2')
        
        self.ikEnv = self.simIK.createEnvironment()
        simBase = self.sim.getObject('./ROBOTIQ85')
        
        self.ikGroup1 = self.simIK.createIkGroup(self.ikEnv)
        simTip1 = self.sim.getObject('./LclosureDummyA')
        simTarget1 = self.sim.getObject('./LclosureDummyB')
        self.simIK.addIkElementFromScene(self.ikEnv, self.ikGroup1, simBase, simTip1, simTarget1, self.simIK.constraint_x + self.simIK.constraint_z)
        
        self.ikGroup2 = self.simIK.createIkGroup(self.ikEnv)
        simTip2 = self.sim.getObject('./RclosureDummyA')
        simTarget2 = self.sim.getObject('./RclosureDummyB')
        self.simIK.addIkElementFromScene(self.ikEnv, self.ikGroup2, simBase, simTip2, simTarget2, self.simIK.constraint_x + self.simIK.constraint_z)

        self.connector = self.sim.getObject('./attachPoint')
        self.objectSensor = self.sim.getObject('./attachProxSensor')
    
    def setActuationType(self, close, shapePath = './Cuboid'):
        self.close = close
        shape = self.sim.getObject(shapePath)
        if self.close:
            if self.sim.checkProximitySensor(self.objectSensor, shape)[0] == 1:
                self.sim.setObjectParent(shape, self.connector, True)
        else:
            self.sim.setObjectParent(shape, -1, True)
    
    def actuation(self):
        p1 = self.sim.getJointPosition(self.j1)
        p2 = self.sim.getJointPosition(self.j2)
        if (self.close):
            if (p1<p2-0.008):
                self.sim.setJointTargetVelocity(self.j1, -0.01)
                self.sim.setJointTargetVelocity(self.j2, -0.04)
            else:
                self.sim.setJointTargetVelocity(self.j1, -0.04)
                self.sim.setJointTargetVelocity(self.j2, -0.04)
        else:
            if (p1<p2):
                self.sim.setJointTargetVelocity(self.j1, 0.04)
                self.sim.setJointTargetVelocity(self.j2, 0.02)
            else:
                self.sim.setJointTargetVelocity(self.j1, 0.02)
                self.sim.setJointTargetVelocity(self.j2, 0.04)                
        
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup1)
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup2)
            
class GripperChildScript:
    def __init__(self, client, sim, gripperName = './ROBOTIQ85'):
        self.client = client
        self.sim = sim
        
        handle = self.sim.getObject(gripperName)
        self.scriptHandle = self.sim.getScript(self.sim.scripttype_childscript, handle)
        self.connector = self.sim.getObject('./attachPoint')
        self.objectSensor = self.sim.getObject('./attachProxSensor')
    
    def actuation(self, close):
        self.sim.callScriptFunction('Actuation', self.scriptHandle, close)
        
    def open(self):
        self.sim.setObjectParent(self.shape, -1, True)
        self.actuation(False)
        self.client.step()
        
    def close(self, shapeName = './Cuboid'):
        self.actuation(True)
        self.shape = self.sim.getObject(shapeName)
        if self.sim.checkProximitySensor(self.objectSensor,self.shape)[0] == 1:
            self.sim.setObjectParent(self.shape, self.connector, True)
        self.client.step()
