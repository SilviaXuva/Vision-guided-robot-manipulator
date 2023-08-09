import numpy as np
import time
from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

class CoppeliaRobot():
    def __init__(self, robot) -> None:
        self.robot = robot
        
        self.startSimulation = sim.startSimulation
        self.stopSimulation = sim.stopSimulation

        self.stopSimulation()
        time.sleep(2)
        self.getRobotHandle()
        self.getJoints()

    def getRobotHandle(self):
        self.robot.handle = sim.getObject(f'./{self.robot.name}')

    def getJoints(self):
        self.robot.joints = list()
        for i in range(0, self.robot.numberJoints):
            handle = sim.getObject(f'./joint{i+1}')
            self.robot.joints.append(handle)

    def getJointPosition(self):
        q = []
        for i in range(0, self.robot.numberJoints):
            q.append(sim.getJointPosition(self.robot.joints[i]))
        return q

    def setJointTargetVelocity(self, vel):
        client.setStepping(True)
        
        for i in range(0, self.robot.numberJoints):
            sim.setJointTargetVelocity(self.robot.joints[i], np.float64(vel[i]))
            
        client.step()
        
    def setJointTargetPosition(self, pos):
        client.setStepping(True)
        
        for i in range(0, self.robot.numberJoints):
            sim.setJointTargetPosition(self.robot.joints[i], np.float64(pos[i]))
                    
        client.step()
        
    # def Wait(timeout = 100):
    #     # startTime = sim.getSimulationTime()
    #     # while sim.getSimulationTime() - startTime < timeout:
    #     #     self.client.step()
    #     i = 0
    #     while i < timeout:
    #         self.client.step()
    #         i = i + 1

    # def SetJointTargetPosition(numberJoints, joints, pos):
    #     for i in range(0, numberJoints):
    #         sim.setJointTargetPosition(joints[i], np.float64(pos[i]))

    # def GetObjectPosition(name, relative = sim.handle_world):
    #     return sim.getObjectPosition(sim.getObject(f'./{name}'), relative)

    # def GetObjectPose(name, relative = sim.handle_world):
    #     return sim.getObjectPose(sim.getObject(f'./{name}'), relative)

    # def BuildPoseYPR(coord, angles):
    #     """Build pose with Yaw Pitch Roll (Euler) angles"""
    #     y, p, r = angles
    #     return sim.buildPose(coord, angles)

    # def BuildPoseABG(coord, angles):
    #     """Build pose with Alfa Beta Gama angles"""
    #     a, b, g = angles
    #     return sim.buildPose(coord, ABG2YPR(angles))

    # def YPR2ABG(angles):
    #     """Yaw Pitch Roll (Euler) angles to Alfa Beta Gama angles"""
    #     y, p, r = angles
    #     return list(sim.yawPitchRollToAlphaBetaGamma(y, p, r))

    # def ABG2YPR(angles):
    #     """Alfa Beta Gama angles to Yaw Pitch Roll (Euler) angles"""
    #     a, b, g = angles
    #     return list(sim.alphaBetaGammaToYawPitchRoll(a, b, g))

    # def GetObjectMatrix(handle, relative = sim.handle_world):
    #     return sim.getObjectMatrix(handle, relative)

    # def ApplyControl(numberJoints, joints, u):
    #     for i in range(0, numberJoints):
    #         sim.setJointTargetVelocity(joints[i], np.float64(u[i]))
    #     self.client.step()

    # startTime = sim.getSimulationTime()
    # while sim.getSimulationTime() - startTime < timeout:
    #     self.client.step()

    # function ApplyControl(obj, u, delta_t)
    #     for i = 1:7
    #         obj.sim.self.simxSetJointTargetVelocity(obj.self.clientID, obj.robot_joints(i), u(i), obj.sim.self.simx_opmode_oneshot);
    #     end
    #     for i = 1:(delta_t/obj.step_time_vrep)				%Number of integrations in delta_t
    #         obj.sim.self.simxSynchronousTrigger(obj.self.clientID);	%Triggering the integration
    #         % To overcome delay in values according to (Remote API modus operandi) document  
    #     end
    #     obj.sim.self.simxGetPingTime(obj.self.clientID);				%Synchronizing
    # end
