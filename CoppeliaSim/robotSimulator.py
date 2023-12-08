import numpy as np
import os
from CoppeliaSim.drawing import Drawing, clearDrawing
from CoppeliaSim.gripper import GripperChildScript, RobotiqGripper
from CoppeliaSim.vision import VisionNonThreaded, VisionThreaded
from zmqRemoteApi import RemoteAPIClient
from settings import Settings

class RobotSimulator:
    def __init__(self, robot, scene = 'main_scene.ttt', drawing = False, gripper = False, vision = False) -> None:
        self.robot = robot
        
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.stop()
        try:
            self.getRobotHandle()
        except:
            print(f'Loading scene {scene}')
            self.sim.loadScene(fr'C:\Users\silvi\Documents\UFSCar\TCC\Python\My-repositories\Files\Scenes\{scene}')
        self.getJoints()
        self.lockJoints()
        
        if drawing:
            self.Drawing = Drawing(self.client, self.sim)
        if gripper:
            self.Gripper = RobotiqGripper(self.client, self.sim)
        if vision:
            self.Vision = VisionNonThreaded(self.client, self.sim)
            
        self.robot.q = self.getJointsPosition()


    def step(self, x_ref = None):     
        if hasattr(self, 'Drawing'):
            self.Drawing.show(x_ref)

        if hasattr(self, 'Vision'):
            try:
                self.Vision.getImg()
                self.Vision.preProcessing()
                self.Vision.detectAruco()
                self.Vision.drawAruco()
                self.Vision.drawArucoPose()
            except Exception as e:
                Settings.log('While processing ArUco:', repr(e))
            self.Vision.showImg()

        self.client.step()
    
    def start(self):
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.client.setStepping(True)
        self.sim.startSimulation()
        print('Simulation started')
        self.client.step()
        self.step()

    def stop(self):
        clearDrawing(self.sim)
        self.sim.stopSimulation()
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 8)
        print('Simulation stopped')

    def getRobotHandle(self):
        self.robot.handle = self.sim.getObject(f'./{self.robot.name}')

    def getJoints(self):
        self.robot.joints = list()
        for i in range(0, self.robot.number_joints):
            handle = self.sim.getObject(f'./joint{i}')
            self.robot.joints.append(handle)
            
    def lockJoints(self):
        for joint in self.robot.joints:
            self.sim.setObjectInt32Param(joint, self.sim.jointintparam_velocity_lock, 1)

    def getJointsPosition(self):
        q = []
        for joint in self.robot.joints:
            q.append(self.sim.getJointPosition(joint))
        return q

    def setJointsTargetVelocity(self, vel):
        for i, joint in enumerate(self.robot.joints):
            self.sim.setJointTargetVelocity(joint, np.float64(vel[i]))
