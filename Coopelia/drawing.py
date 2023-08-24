class Drawing:
    def __init__(self, client, sim) -> None:
        self.client = client
        self.sim = sim
        
        self.handle = self.sim.getObject('./tip')
        self.currentDrawing = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, [1, 0, 0])
        self.referenceDrawing = self.sim.addDrawingObject(self.sim.drawing_lines|self.sim.drawing_cyclic, 2, 0, -1, 200, [0, 1, 1])
        self.currentPosition = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        self.referencePosition = self.currentPosition
        
        f = open(r'.\Output\drawing.log', "w", encoding='utf-8')
        f.write(f'sim.addDrawingObjectItem({self.currentDrawing}, nil)' + '\n' + f'sim.addDrawingObjectItem({self.referenceDrawing}, nil)')
        f.close()
        
    def show(self, reference):
        currentLine = self.currentPosition
        self.currentPosition = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        for pos in self.currentPosition:
            currentLine.append(pos)
        self.sim.addDrawingObjectItem(self.currentDrawing, currentLine)
        referenceLine = self.referencePosition
        for pos in reference:
            referenceLine.append(pos)
        self.referencePosition = reference
        self.sim.addDrawingObjectItem(self.referenceDrawing, referenceLine)
        
    def clear(self):
        self.sim.addDrawingObjectItem(self.currentDrawing, None)
        self.sim.addDrawingObjectItem(self.referenceDrawing, None)
