import numpy as np
import time
from zmqRemoteApi import RemoteAPIClient

class CoppeliaRobot():
    def __init__(self, robot) -> None:
        self.robot = robot

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        self.start = self.sim.startSimulation
        self.stop = self.sim.stopSimulation

        self.stop()
        time.sleep(2)
        self.getRobotHandle()
        self.getJoints()
        
        self.client.setStepping(True)

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
        for i in range(0, self.robot.numberJoints):
            self.sim.setJointTargetVelocity(self.robot.joints[i], np.float64(vel[i]))
        self.client.step()
        
    def setJointTargetPosition(self, pos):
        for i in range(0, self.robot.numberJoints):
            self.sim.setJointTargetPosition(self.robot.joints[i], np.float64(pos[i]))
        self.client.step()
        
    # def Wait(self, timeout = 100):
    #     startTime = self.sim.getSimulationTime()
    #     while self.sim.getSimulationTime() - startTime < timeout:
    #         self.client.step()
    #     i = 0
    #     while i < timeout:
    #         self.client.step()
    #         i = i + 1

    # def SetJointTargetPosition(self, numberJoints, joints, pos):
    #     for i in range(0, numberJoints):
    #         self.sim.setJointTargetPosition(joints[i], np.float64(pos[i]))

    # def GetObjectPosition(self, name, relative = None):
    #     if relative is None:
    #         relative = self.sim.handle_world
    #     return self.sim.getObjectPosition(self, self.sim.getObject(f'./{name}'), relative)

    # def GetObjectPose(self, name, relative = None):
    #     if relative is None:
    #         relative = self.sim.handle_world
    #     return self.sim.getObjectPose(self.sim.getObject(f'./{name}'), relative)

    # def BuildPoseYPR(self, coord, angles):
    #     """Build pose with Yaw Pitch Roll (Euler) angles"""
    #     y, p, r = angles
    #     return self.sim.buildPose(coord, angles)

    # def BuildPoseABG(self, coord, angles):
    #     """Build pose with Alfa Beta Gama angles"""
    #     a, b, g = angles
    #     return self.sim.buildPose(coord, self.ABG2YPR(angles))

    # def YPR2ABG(self, angles):
    #     """Yaw Pitch Roll (Euler) angles to Alfa Beta Gama angles"""
    #     y, p, r = angles
    #     return list(self.sim.yawPitchRollToAlphaBetaGamma(y, p, r))

    # def ABG2YPR(self, angles):
    #     """Alfa Beta Gama angles to Yaw Pitch Roll (Euler) angles"""
    #     a, b, g = angles
    #     return list(self.sim.alphaBetaGammaToYawPitchRoll(a, b, g))

    # def GetObjectMatrix(self, handle, relative = None):
    #     if relative is None:
    #         relative = self.sim.handle_world
    #     return self.sim.getObjectMatrix(handle, relative)

    # def ApplyControl(self, numberJoints, joints, u):
    #     for i in range(0, numberJoints):
    #         self.sim.setJointTargetVelocity(joints[i], np.float64(u[i]))
    #     self.client.step()

    # # startTime = sim.getSimulationTime()
    # # while sim.getSimulationTime() - startTime < timeout:
    # #     self.client.step()

    # # function ApplyControl(obj, u, delta_t)
    # #     for i = 1:7
    # #         obj.sim.self.simxSetJointTargetVelocity(obj.self.clientID, obj.robot_joints(i), u(i), obj.sim.self.simx_opmode_oneshot);
    # #     end
    # #     for i = 1:(delta_t/obj.step_time_vrep)				%Number of integrations in delta_t
    # #         obj.sim.self.simxSynchronousTrigger(obj.self.clientID);	%Triggering the integration
    # #         % To overcome delay in values according to (Remote API modus operandi) document  
    # #     end
    # #     obj.sim.self.simxGetPingTime(obj.self.clientID);				%Synchronizing
    # # end