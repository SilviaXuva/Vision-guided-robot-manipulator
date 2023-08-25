import numpy as np
import os
from Coopelia.gripper import GripperChildScript, RobotiqGripper
from Coopelia.drawing import Drawing
from zmqRemoteApi import RemoteAPIClient

class RobotSimulator:
    def __init__(self, robot, scene = 'simulation_vel_camera_without_childscript.ttt', drawing = False, gripper = False, vision = False) -> None:
        self.robot = robot
        
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        self.stop()
        try:
            self.getRobotHandle()
        except:
            self.sim.loadScene(fr"{os.path.abspath(os.curdir)}\Scenes\{scene}")
        self.getJoints()
        self.lockJoints()
        
        if drawing:
            self.Drawing = Drawing(self.client, self.sim)
        if gripper:
            self.Gripper = RobotiqGripper(self.client, self.sim)
        if vision:
            self.Vision = None
        
        self.robot.q = self.getJointsPosition()
        self.q = list()

    def step(self):
        self.client.step()
    
    def start(self):
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.client.setStepping(True)
        self.sim.startSimulation()

    def stop(self):
        if hasattr(self, 'Drawing'):
            self.Drawing.clear()
        self.sim.stopSimulation()
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 8)

    def getJointsPosition(self):
        q = []
        for i in range(0, self.robot.number_joints):
            q.append(self.sim.getJointPosition(self.robot.joints[i]))
        return q

    def setJointsTargetVelocity(self, vel):
        for i in range(0, self.robot.number_joints):
            self.sim.setJointTargetVelocity(self.robot.joints[i], np.float64(vel[i]))

    def getRobotHandle(self):
        self.robot.handle = self.sim.getObject(f'./{self.robot.name}')

    def getJoints(self):
        self.robot.joints = list()
        for i in range(0, self.robot.number_joints):
            handle = self.sim.getObject(f'./joint{i+1}')
            self.robot.joints.append(handle)
            
    def lockJoints(self):
        for i in range(0, self.robot.number_joints):
            self.sim.setObjectInt32Param(self.robot.joints[i], self.sim.jointintparam_velocity_lock, 1)
