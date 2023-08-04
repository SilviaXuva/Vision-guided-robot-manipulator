import numpy as np
import time
from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

class Coppelia():
    def __init__(self, robot = None) -> None:
        self.robot = robot
        
        self.client = client
        self.sim = self.client.getObject('sim')
        self.startSimulation = self.sim.startSimulation
        self.stopSimulation = self.sim.stopSimulation

        self.stopSimulation()
        time.sleep(2)
        self.getRobotHandle()
        self.getJoints()
        
    def getRobotHandle(self):
        self.robot.handle = self.sim.getObject(f'./{self.robot.name}')

    def getJoints(self):
        self.robot.joints = list()
        for i in range(0, self.robot.numberJoints):
            handle = self.sim.getObject(f'./joint{i+1}')
            self.robot.joints.append(handle)

    def getJointPosition(self):
        q = []
        for i in range(0, self.robot.numberJoints):
            q.append(self.sim.getJointPosition(self.robot.joints[i]))
        return q

    def setJointTargetVelocity(self, vel):
        self.client.setStepping(True)
        
        for i in range(0, self.robot.numberJoints):
            self.sim.setJointTargetVelocity(self.robot.joints[i], np.float64(vel[i]))
            
        self.client.step()
        
    def setJointTargetPosition(self, pos):
        self.client.setStepping(True)
        
        for i in range(0, self.robot.numberJoints):
            self.sim.setJointTargetPosition(self.robot.joints[i], np.float64(pos[i]))
                    
        self.client.step()
        
    # def Wait(timeout = 100):
    #     # startTime = self.sim.getSimulationTime()
    #     # while self.sim.getSimulationTime() - startTime < timeout:
    #     #     self.client.step()
    #     i = 0
    #     while i < timeout:
    #         self.client.step()
    #         i = i + 1

    # def SetJointTargetPosition(numberJoints, joints, pos):
    #     for i in range(0, numberJoints):
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

    # def ApplyControl(numberJoints, joints, u):
    #     for i in range(0, numberJoints):
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
