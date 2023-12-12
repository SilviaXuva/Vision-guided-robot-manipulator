from settings import Settings, Cuboids as Cb
from Simulators.CoppeliaSim.objects import CoppeliaObj

import random
import numpy as np
import math

class Cuboids(CoppeliaObj):
    def __init__(self, sim) -> None:
        super().__init__(sim, 'Cuboids')
        self.handle = self.sim.getObjectHandle(Cb.path)
        self.bodyHandle = self.sim.getObjectHandle(Cb.bodyPath)
        self.colors={
            'red': [1,0,0],
            'green': [0,1,0],
            'blue': [0,0,1],
        }

    def CreateCuboid(self):
        markerId = random.randrange(1,10)
        color = random.choice(list(self.colors.items()))
        inserted = self.sim.copyPasteObjects(
            [self.handle, self.sim.getObjectHandle(Cb.markerPath.replace('{id}', str(markerId))), self.bodyHandle]
            ,0
        )

        self.sim.setObjectAlias(inserted[1], self.sim.getObjectAlias(inserted[1]).replace("ref_",""))
        self.sim.setShapeColor(inserted[1], None, self.sim.colorcomponent_ambient_diffuse, color[1])
        self.sim.setObjectAlias(inserted[2], self.sim.getObjectAlias(inserted[2]).replace("ref_",""))
        self.sim.setShapeColor(inserted[2], None, self.sim.colorcomponent_ambient_diffuse, color[1])

        rz = random.uniform(-np.pi, np.pi)
        y = random.uniform(-0.2, 0.2)
        self.sim.setObjectAlias(inserted[0], color[0] + str(markerId))
        self.sim.setObjectPosition(inserted[0], -1, [1, y, 0.23])
        self.sim.setObjectOrientation(inserted[0], -1, [0, 0, rz])
