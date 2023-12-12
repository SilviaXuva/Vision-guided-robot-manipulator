from settings import Settings
from Simulators.CoppeliaSim.objects import CoppeliaObj

import numpy as np

red = [1, 0, 0]
green = [0, 1, 0]
blue = [0, 0, 1]

class Drawing(CoppeliaObj):
    def __init__(self, sim) -> None:
        super().__init__(sim, 'Drawing')
        self.tipHandle = self.sim.getObject('./tip')
        self.Ref = self.Ref(self.sim, self.tipHandle)
        self.Real = self.Real(self.sim, self.tipHandle)

    def Show(self, refPos: np.ndarray = None):
        self.Real.UpdateLine()
        if refPos is not None:
            self.Ref.UpdateLine(refPos)

    class DrawingObject():
        def __init__(self, sim, tipHandle: str, color: list) -> None:
            self.sim = sim
            self.tipHandle = tipHandle
            self.obj = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, color)
            self.pos = np.array(self.sim.getObjectPosition(self.tipHandle, self.sim.handle_world))

        def UpdateLine(self, pos: np.ndarray):
            pos = np.array(pos)
            line = np.concatenate([self.pos, pos])
            self.sim.addDrawingObjectItem(self.obj, line.tolist())
            # x_axis = np.concatenate([pos, np.array([pos[0]+0.05, pos[1], pos[2]])])
            # y_axis = np.concatenate([pos, np.array([pos[0], pos[1]+0.05, pos[2]])])
            # z_axis = np.concatenate([pos, np.array([pos[0], pos[1], pos[2]+0.05])])
            # self.sim.addDrawingObjectItem(self.obj, x_axis.tolist())
            # self.sim.addDrawingObjectItem(self.obj, y_axis.tolist())
            # self.sim.addDrawingObjectItem(self.obj, z_axis.tolist())
            self.pos = pos
            
    class Ref(DrawingObject):
        def __init__(self, sim, tipHandle: str, color: list = blue) -> None:
            super().__init__(sim, tipHandle, color)

    class Real(DrawingObject):
        def __init__(self, sim, tipHandle: str, color: list = red) -> None:
            super().__init__(sim, tipHandle, color)

        def UpdateLine(self):
            super().UpdateLine(self.sim.getObjectPosition(self.tipHandle, self.sim.handle_world))

def ClearDrawing(sim):
    Settings.Log('Clear drawing...')
    for i in range(1, 100):
        try:
            sim.addDrawingObjectItem(i, None)
            sim.removeDrawingObject(i, None)
        except:
            pass