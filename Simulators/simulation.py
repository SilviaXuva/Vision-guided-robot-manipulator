from Model.settings import Settings

from Simulators.PyPlot.pyPlotRobot import PyPlotRobot
from Simulators.Coppelia.coppeliaRobot import CoppeliaRobot
from Simulators.Coppelia.gripper import GripperChildScript, RobotiqGripper
from Simulators.Coppelia.vision import Vision

class Simulator():
    def __init__(self, robot) -> None:
        self.robot = robot
        
        if 'coppelia' in Settings.environments:
            self.coppelia = CoppeliaRobot(robot)
            self.client = self.coppelia.client
            self.sim = self.coppelia.sim
            if Settings.gripper:
                self.Gripper = RobotiqGripper(self.coppelia.client, self.coppelia.sim)
            if Settings.vision:
                self.Vision = Vision(self.coppelia.client, self.coppelia.sim)
        else:
            self.coppelia = None
        if 'pyplot' in Settings.environments:
            self.pyplot = PyPlotRobot(robot)
        else:
            self.pyplot = None
    
    def start(self):
        if self.coppelia is not None:
            self.coppelia.start()
        if self.pyplot is not None:
            self.pyplot.start()

    def stop(self):
        if self.coppelia is not None:
            self.coppelia.stop()
        if self.pyplot is not None:
            self.pyplot.stop()

    def getJointPosition(self):
        if self.coppelia is not None:
            q = self.coppelia.getJointPosition()
        else:
            if self.pyplot is not None:
                q = self.pyplot.getJointPosition()
        return q

    def setJointTargetVelocity(self, vel):
        if self.pyplot is not None:
            self.pyplot.setJointTargetVelocity(vel)
        if self.coppelia is not None:
            self.coppelia.setJointTargetVelocity(vel)
        
    def setJointTargetPosition(self, pos):
        if self.pyplot is not None:
            self.pyplot.setJointTargetPosition(pos)
        if self.coppelia is not None:
            self.coppelia.setJointTargetPosition(pos)

    def NoneFunc(self, *args, **kwargs):
        pass

    def __getattr__(self, name):
        # assume it is implemented by self.instance
        try:
            return self.pyplot.__getattribute__(name)
        except:
            return self.NoneFunc
