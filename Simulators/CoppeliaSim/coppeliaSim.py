from settings import Settings
from Simulators.sim import Sim
from Simulators.CoppeliaSim import Drawing, ClearDrawing, RobotiqGripper, Camera, Conveyor, ProximitySensor, Cuboids
from VisionProcessing.aruco import ArucoVision

import numpy as np
from zmqRemoteApi import RemoteAPIClient

class CoppeliaSim(Sim):
    def __init__(self, 
            robot, 
            scene: str = 'main_scene.ttt', 
            drawing: bool = False, 
            gripper: bool = False, 
            camera: bool = False, 
            arucoVision: bool = False,
            createCuboids: bool = False
        ) -> None:
        Sim.__init__(self, robot)

        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        self.Stop()
        try:
            self.GetRobotHandle()
        except:
            Settings.Log(f'Loading scene {scene}')
            self.sim.loadScene(fr'C:\Users\silvi\Documents\UFSCar\TCC\Python\My-repositories\Files\Scenes\{scene}')
        self.GetJoints()
        self.LockJoints()

        if drawing:
            self.Drawing = Drawing(self.sim)
        if gripper:
            self.Gripper = RobotiqGripper(self.client, self.sim)
        if camera:
            self.Camera = Camera(self.sim)
        if arucoVision:
            self.ArucoVision = ArucoVision(self.Camera)
        if createCuboids:
            self.Conveyor = Conveyor(self.sim)
            self.ProximitySensor = ProximitySensor(self.sim)
            self.Cuboids = Cuboids(self.sim)
            self.moveRobot = False
        else:
            self.moveRobot = True

    def Start(self):
        Sim.Start(self, 'Coppelia')
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.client.setStepping(True)
        self.sim.startSimulation()
        self.client.step()

    def Stop(self):
        Sim.Stop(self, 'Coppelia')
        ClearDrawing(self.sim)
        self.sim.stopSimulation()
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 8)

    def GetRobotHandle(self):
        self.robot.handle = self.sim.getObject(f'./{self.robot.name}')

    def GetJoints(self):
        self.robot.joints = list()
        for i in range(0, self.robot.n):
            handle = self.sim.getObject(f'./joint{i}')
            self.robot.joints.append(handle)
            
    def LockJoints(self):
        for joint in self.robot.joints:
            self.sim.setObjectInt32Param(joint, self.sim.jointintparam_velocity_lock, 1)

    def GetJointsPosition(self):
        q = []
        for joint in self.robot.joints:
            q.append(self.sim.getJointPosition(joint))
        return np.array(q)

    def SetJointsTargetVelocity(self, vel: list):
        for i, joint in enumerate(self.robot.joints):
            self.sim.setJointTargetVelocity(joint, np.float64(vel[i]))

    def Step(self, xRef: np.ndarray = None): 
        if hasattr(self, 'Drawing'):
            self.Drawing.Show(xRef)

        if hasattr(self, 'Camera'):
            self.Camera.GetImg()
        
        if hasattr(self, 'ArucoVision'):
            self.ArucoVision.Process()
            self.ArucoVision.ShowImg()
        
        if hasattr(self, 'Conveyor'):
            if self.ProximitySensor.CheckProximity():
                self.Conveyor.Move(0)
                if self.Conveyor.CheckStopped():
                    self.moveRobot = True
            else:
                self.Conveyor.Move(0.1)
            if self.ProximitySensor.CallCuboidCreation() and self.Cuboids.created < self.Cuboids.maxCreation:
                self.Cuboids.CreateCuboid()

        self.client.step()