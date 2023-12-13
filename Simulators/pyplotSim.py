from settings import Settings
from Simulators.sim import Sim

import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot

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

    def SetJointsTargetVelocity(self, vel: np.ndarray):
        self.robot.qd = vel

    def Step(self, xRef: np.ndarray = None): 
        self.step(Settings.Ts)