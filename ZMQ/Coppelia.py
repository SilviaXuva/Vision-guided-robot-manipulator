import numpy as np
from ZMQ.zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

def stopSimulation():
    sim.stopSimulation()
    
def startSimulation():
    sim.startSimulation()

class Coppelia():
    def __init__(self) -> None:
        self.client = client
        self.sim = sim
        self.startSimulation = startSimulation
        self.stopSimulation = stopSimulation
        
    def getRobotHandle(self):
        return self.sim.getObject(f'./{self.model}')

    def getJoints(self):
        joints = list()
        for i in range(0, self.number_joints):
            handle = self.sim.getObject(f'./joint{i+1}')
            joints.append(handle)
        return joints

    def getJointPosition(self):
        q = []
        for i in range(0, self.number_joints):
            q.append(self.sim.getJointPosition(self.joints[i]))
        return q

    def setJointPosition(self, pos):
        for i in range(0, self.number_joints):
            self.sim.setJointPosition(self.joints[i], np.float64(pos[i]))
        
    def setJointTargetVelocity(self, vel):
        self.client.setStepping(True)
        
        for i in range(0, self.number_joints):
            self.sim.setJointTargetVelocity(self.joints[i], np.float64(vel[i]))
            
        self.client.step()
        
    def setJointTargetPosition(self, pos):
        self.client.setStepping(True)
        
        for i in range(0, self.number_joints):
            self.sim.setJointTargetPosition(self.joints[i], np.float64(pos[i]))
                    
        self.client.step()
        
    # def Wait(timeout = 100):
    #     # startTime = self.sim.getSimulationTime()
    #     # while self.sim.getSimulationTime() - startTime < timeout:
    #     #     self.client.step()
    #     i = 0
    #     while i < timeout:
    #         self.client.step()
    #         i = i + 1

    # def SetJointTargetPosition(number_joints, joints, pos):
    #     for i in range(0, number_joints):
    #         self.sim.setJointTargetPosition(joints[i], np.float64(pos[i]))

    # def GetObjectPosition(name, relative = self.sim.handle_world):
    #     return self.sim.getObjectPosition(self.sim.getObject(f'./{name}'), relative)

    # def GetObjectPose(name, relative = self.sim.handle_world):
    #     return self.sim.getObjectPose(self.sim.getObject(f'./{name}'), relative)

    # def BuildPoseYPR(coord, angles):
    #     """Build pose with Yaw Pitch Roll (Euler) angles"""
    #     y, p, r = angles
    #     return self.sim.buildPose(coord, angles)

    # def BuildPoseABG(coord, angles):
    #     """Build pose with Alfa Beta Gama angles"""
    #     a, b, g = angles
    #     return self.sim.buildPose(coord, ABG2YPR(angles))

    # def YPR2ABG(angles):
    #     """Yaw Pitch Roll (Euler) angles to Alfa Beta Gama angles"""
    #     y, p, r = angles
    #     return list(self.sim.yawPitchRollToAlphaBetaGamma(y, p, r))

    # def ABG2YPR(angles):
    #     """Alfa Beta Gama angles to Yaw Pitch Roll (Euler) angles"""
    #     a, b, g = angles
    #     return list(self.sim.alphaBetaGammaToYawPitchRoll(a, b, g))

    # def GetObjectMatrix(handle, relative = self.sim.handle_world):
    #     return self.sim.getObjectMatrix(handle, relative)

    # def ApplyControl(number_joints, joints, u):
    #     for i in range(0, number_joints):
    #         self.sim.setJointTargetVelocity(joints[i], np.float64(u[i]))
    #     self.client.step()

        # startTime = self.sim.getSimulationTime()
        # while self.sim.getSimulationTime() - startTime < timeout:
        #     self.client.step()

        # function ApplyControl(obj, u, delta_t)
        #     for i = 1:7
        #         obj.self.sim.self.simxSetJointTargetVelocity(obj.self.clientID, obj.robot_joints(i), u(i), obj.self.sim.self.simx_opmode_oneshot);
        #     end
        #     for i = 1:(delta_t/obj.step_time_vrep)				%Number of integrations in delta_t
        #         obj.self.sim.self.simxSynchronousTrigger(obj.self.clientID);	%Triggering the integration
        #         % To overcome delay in values according to (Remote API modus operandi) document  
        #     end
        #     obj.self.sim.self.simxGetPingTime(obj.self.clientID);				%Synchronizing
        # end
