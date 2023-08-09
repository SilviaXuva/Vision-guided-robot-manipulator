from Model.Settings import environments, gripper

from Simulators.PyPlot.PyPlotRobot import PyPlotRobot
from Simulators.Coppelia.CoppeliaRobot import CoppeliaRobot
from Simulators.Coppelia.Gripper import Gripper_ChildScript

class Simulators():
    def __init__(self) -> None:
        if 'coppelia' in environments:
            self.coppelia = CoppeliaRobot(self)
            if gripper:
                self.Gripper = Gripper_ChildScript()
        if 'toolbox' in environments:
            self.pyplot = PyPlotRobot(self)
    
    def startSimulation(self):
        if self.coppelia:
            self.coppelia.startSimulation()
        if self.pyplot:
            self.pyplot.startSimulation()

    def stopSimulation(self):
        if self.coppelia:
            self.coppelia.stopSimulation()
        if self.pyplot:
            self.pyplot.stopSimulation()

    def getJointPosition(self):
        if self.coppelia:
            q = self.coppelia.getJointPosition()
        else:
            if self.pyplot:
                q = self.pyplot.getJointPosition()
        return q

    def setJointTargetVelocity(self, vel):
        if self.pyplot:
            self.pyplot.setJointTargetVelocity(vel)
        if self.coppelia:
            self.coppelia.setJointTargetVelocity(vel)
        
    def setJointTargetPosition(self, pos):
        if self.pyplot:
            self.pyplot.setJointTargetPosition(pos)
        if self.coppelia:
            self.coppelia.setJointTargetPosition(pos)

    def __getattr__(self, name):
        # assume it is implemented by self.instance
        try:
            return self.pyplot.__getattribute__(name)
        except:
            pass
