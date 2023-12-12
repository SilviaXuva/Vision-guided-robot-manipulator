from Simulators.sim import Sim
from settings import Settings

from swift import Swift
import numpy as np

class SwiftSim(Sim, Swift):
    def __init__(self, robot) -> None:
        Sim.__init__(self, robot)
        Swift.__init__(self)

    def Start(self):
        Sim.Start(self, 'Swift')
        self.launch()
        self.add(self.robot)
    
    def Stop(self):
        Sim.Stop(self, 'Swift')
    
    def GetJointsPosition(self):
        return self.robot.q

    def SetJointsTargetVelocity(self, vel: list):
        self.robot.qd = vel

    def Step(self, xRef: np.ndarray = None): 
        self.step(Settings.Ts)