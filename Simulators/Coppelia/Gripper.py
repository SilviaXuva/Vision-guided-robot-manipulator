from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

class Gripper_ChildScript():
    def __init__(self, gripperName = './ROBOTIQ85'):
        self.sim = sim
        
        gripper = self.sim.getObject(gripperName)
        self.scriptHandle = self.sim.getScript(self.sim.scripttype_childscript, gripper)
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

# class RobotiqGripper():
#     def __init__(self):
#         self.j1 = sim.getObject('./active1')
#         self.j2 = sim.getObject('./active2')
        
#         self.ikEnv = simIK.createEnvironment()
#         simBase = sim.getObject('./ROBOTIQ85')
        
#         self.ikGroup1 = simIK.createIkGroup(self.ikEnv)
#         simTip1 = sim.getObject('./LclosureDummyA')
#         simTarget1 = sim.getObject('./LclosureDummyB')
#         simIK.addIkElementFromScene(self.ikEnv, self.ikGroup1, simBase, simTip1, simTarget1, simIK.constraint_x + simIK.constraint_z)
        
#         self.ikGroup2 = simIK.createIkGroup(self.ikEnv)
#         simTip2 = sim.getObject('./RclosureDummyA')
#         simTarget2 = sim.getObject('./RclosureDummyB')
#         simIK.addIkElementFromScene(self.ikEnv, self.ikGroup2, simBase, simTip2, simTarget2, simIK.constraint_x + simIK.constraint_z)
        
#         self.Actuation(False)
        
#         self.connector = sim.getObject('./attachPoint')
    
#     def Actuation(self, close, abs_tol = 0.003):
#         p1 = sim.getJointPosition(self.j1)
#         p2 = sim.getJointPosition(self.j2)
#         if (close):
#             if (p1<p2-0.008):
#                 sim.setJointTargetVelocity(self.j1, -0.01)
#                 sim.setJointTargetVelocity(self.j2, -0.04)
#             else:
#                 sim.setJointTargetVelocity(self.j1, -0.04)
#                 sim.setJointTargetVelocity(self.j2, -0.04)
#         else:
#             if (p1<p2):
#                 sim.setJointTargetVelocity(self.j1, 0.04)
#                 sim.setJointTargetVelocity(self.j2, 0.02)
#             else:
#                 sim.setJointTargetVelocity(self.j1, 0.02)
#                 sim.setJointTargetVelocity(self.j2, 0.04)
        
#         simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup1)
#         simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup2)
        
#         startTime = sim.getSimulationTime()
#         while sim.getSimulationTime() - startTime < 5:
#             client.step()

#     def Open(self):
#         sim.setObjectParent(self.shape, -1, True)
#         self.Actuation(False)
        
#     def Close(self, shapeName = './Cuboid'):
#         self.shape = sim.getObject(shapeName)
#         sim.setObjectParent(self.shape, self.connector, True)
#         self.Actuation(True)