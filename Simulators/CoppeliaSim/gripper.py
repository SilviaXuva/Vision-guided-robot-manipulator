from settings import Settings
from Simulators.CoppeliaSim.objects import CoppeliaObj

class Actuation:
    def __init__(self, actuation: str = None, shapePath: str = None) -> None:
        if actuation == 'close':
            self.close = True
        elif actuation == 'open':
            self.close = False
        else:
            self.close = None
        self.shapePath = shapePath
    
    def Log(self):
        return self.__dict__

class RobotiqGripper(CoppeliaObj):
    def __init__(self, client, sim) -> None:
        super().__init__(sim, 'RobotiqGripper')
        self.client = client
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
    
    def SetupActuation(self, Actuation: Actuation):
        self.close = Actuation.close
        shape = self.sim.getObject(Actuation.shapePath)
        if self.close:
            if self.sim.checkProximitySensor(self.objectSensor, shape)[0] == 1:
                self.sim.setObjectParent(shape, self.connector, True)
                return True
            else:
                return False
        else:
            self.sim.setObjectParent(shape, -1, True)
        return True
    
    def Actuation(self):
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
    def __init__(self, client, sim, gripper_name = './ROBOTIQ85'):
        Settings.Log('Init Gripper...')
        self.client = client
        self.sim = sim
        
        handle = self.sim.getObject(gripper_name)
        self.script_handle = self.sim.getScript(self.sim.scripttype_childscript, handle)
        self.connector = self.sim.getObject('./attachPoint')
        self.objectSensor = self.sim.getObject('./attachProxSensor')
    
    def actuation(self, close):
        self.sim.callScriptFunction('Actuation', self.script_handle, close)
        
    def open(self):
        self.sim.setObjectParent(self.shape, -1, True)
        self.actuation(False)
        self.client.step()
        
    def close(self, shape_name = './Cuboid'):
        self.actuation(True)
        self.shape = self.sim.getObject(shape_name)
        if self.sim.checkProximitySensor(self.objectSensor,self.shape)[0] == 1:
            self.sim.setObjectParent(self.shape, self.connector, True)
        self.client.step()
