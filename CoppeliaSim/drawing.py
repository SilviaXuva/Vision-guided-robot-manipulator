import numpy as np

class DrawingObject():
    def __init__(self, sim, tip_handle, color) -> None:
        self.sim = sim
        self.tip_handle = tip_handle
        self.obj = sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, color)
        self.pos = np.array(sim.getObjectPosition(self.tip_handle, self.sim.handle_world))

    def updateLine(self, pos):
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
    def __init__(self, sim, tip_handle, color = [0, 0, 1]) -> None:
        super().__init__(sim, tip_handle, color)

class Real(DrawingObject):
    def __init__(self, sim, tip_handle, color = [1, 0, 0]) -> None:
        super().__init__(sim, tip_handle, color)

    def updateLine(self):
        super().updateLine(self.sim.getObjectPosition(self.tip_handle, self.sim.handle_world))

class Drawing:
    def __init__(self, client, sim) -> None:
        print('Init Drawing...')
        self.client = client
        self.sim = sim
        
        self.tip_handle = self.sim.getObject('./tip')
        self.Ref = Ref(self.sim, self.tip_handle)
        self.Real = Real(self.sim, self.tip_handle)

    def show(self, ref_pos = None):
        self.Real.updateLine()
        if ref_pos is not None:
            self.Ref.updateLine(ref_pos)

def clearDrawing(sim):
    for i in range(1,16):
        try:
            sim.addDrawingObjectItem(i, None)
            sim.removeDrawingObject(i, None)
        except:
            pass