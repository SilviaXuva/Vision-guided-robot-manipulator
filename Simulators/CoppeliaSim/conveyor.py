from settings import Conveyor as Conv
from Simulators.CoppeliaSim.objects import CoppeliaObj

import math

class Conveyor(CoppeliaObj):
    def __init__(self, sim) -> None:
        super().__init__(sim, 'Conveyor')
        self.sim = sim
        self.handle = self.sim.getObjectHandle(Conv.path)

    def Move(self, vel: float):
        self.sim.writeCustomTableData(self.handle,'__ctrl__',{"vel":vel})

    def CheckStopped(self):
        vel = self.sim.readCustomTableData(self.handle,'__state__')['vel']
        if math.isclose(vel, 0, abs_tol=0.0001):
            return True
        return False