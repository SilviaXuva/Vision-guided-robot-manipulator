from settings import Settings, ProximitySensor as PS
from Simulators.CoppeliaSim.objects import CoppeliaObj

class ProximitySensor(CoppeliaObj):
    def __init__(self, sim) -> None:
        super().__init__(sim, 'Proximity Sensor')
        self.handle = self.sim.getObjectHandle(PS.path)
    
    def CheckProximity(self):
        prox = self.sim.readProximitySensor(self.handle)
        if prox[0] == 1:
            return True
        return False