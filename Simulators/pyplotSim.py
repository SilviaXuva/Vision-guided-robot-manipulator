from Simulators.sim import Sim
from settings import Settings

from roboticstoolbox.backends.PyPlot import PyPlot
import numpy as np

class PyPlotSim(Sim, PyPlot):
    def __init__(self, robot) -> None:
        Sim.__init__(self, robot)
        PyPlot.__init__(self)

    def Start(self):
        Sim.Start(self, 'PyPlot')
        self.launch()
        self.add(self.robot)
    
    def Stop(self):
        Sim.Stop(self, 'PyPlot')

    def GetJointsPosition(self):
        return self.robot.q

    def SetJointsTargetVelocity(self, vel: list):
        self.robot.qd = vel

    def Step(self, xRef: np.ndarray = None): 
        self.step(Settings.Ts)