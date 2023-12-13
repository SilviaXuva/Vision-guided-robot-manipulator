from settings import Cuboids as Cb, Aruco
from Simulators.CoppeliaSim.objects import CoppeliaObj

import numpy as np
import random

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
        self.created = 0
        self.maxCreation = Cb.maxCreation

    def CreateCuboid(self):
        while True:
            markerId = random.randrange(1,10)
            color = random.choice(list(self.colors.items()))
            try:
                self.sim.getObject(f'./{color}{markerId}')
            except:
                break
        
        inserted = self.sim.copyPasteObjects(
            [self.handle, self.sim.getObjectHandle(Cb.markerPath.replace('{id}', str(markerId))), self.bodyHandle],0
        )
        self.sim.setObjectPosition(inserted[1], inserted[0], [0, 0, (Aruco.length/2)])
        self.sim.setObjectPosition(inserted[2], inserted[0], [0,0,0])
        self.sim.setObjectParent(inserted[1], inserted[0], True)
        self.sim.setObjectParent(inserted[2], inserted[0])
        
        rz = random.uniform(-np.pi, np.pi)
        y = random.uniform(-0.2, 0.2)
        self.sim.setObjectPosition(inserted[0], -1, [1, y, 0.225])
        self.sim.setObjectOrientation(inserted[0], -1, [0, 0, rz])
    
        self.sim.setObjectAlias(inserted[1], self.sim.getObjectAlias(inserted[1]).replace("ref_",""))
        self.sim.setShapeColor(inserted[1], None, self.sim.colorcomponent_ambient_diffuse, color[1])
        self.sim.setObjectAlias(inserted[2], self.sim.getObjectAlias(inserted[2]).replace("ref_",""))
        self.sim.setShapeColor(inserted[2], None, self.sim.colorcomponent_ambient_diffuse, color[1])
        self.sim.setObjectAlias(inserted[0], color[0] + str(markerId))
        self.created += 1
