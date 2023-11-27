import os
from settings import Settings

class Drawing:
    def __init__(self, client, sim) -> None:
        self.client = client
        self.sim = sim
        
        self.handle = self.sim.getObject('./tip')
        self.current_drawing = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, [1, 0, 0])
        self.reference_drawing = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, [0, 1, 1])
        self.current_position = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        self.reference_position = self.current_position

        f = open(fr'{Settings.output_path}\drawing.log', "w", encoding='utf-8')
        f.write(f'sim.addDrawingObjectItem({self.current_drawing}, nil)' + '\n' + f'sim.addDrawingObjectItem({self.reference_drawing}, nil)')
        f.close()
        
    def show(self, reference):
        current_line = self.current_position
        self.current_position = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        for pos in self.current_position:
            current_line.append(pos)
        self.sim.addDrawingObjectItem(self.current_drawing, current_line)
        reference_line = self.reference_position
        for pos in reference:
            reference_line.append(pos)
        self.reference_position = reference
        self.sim.addDrawingObjectItem(self.reference_drawing, reference_line)
        
    def clear(self):
        self.sim.addDrawingObjectItem(self.current_drawing, None)
        self.sim.addDrawingObjectItem(self.reference_drawing, None)
        self.sim.removeDrawingObject(self.current_drawing)
        self.sim.removeDrawingObject(self.reference_drawing)
