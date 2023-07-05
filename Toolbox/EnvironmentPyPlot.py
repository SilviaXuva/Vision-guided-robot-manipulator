import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox.backends.PyPlot import PyPlot

class Env(PyPlot):  
    def __init__(self, robot) -> None:
        super().__init__()
        self.robot = robot
    
    def new(self, title = ''):
        """ Initialize new matplotlib plot

        Args:
            robot
            title (str, optional): figure window title. Defaults to ''.
        """
        limits = [-1, 1, -1, 1, 0, 1.5]
        fig = plt.figure()
        self.launch(fig = fig, limits=limits)
        if title != '':
            current = fig.canvas.manager.get_window_title()
            fig.canvas.manager.set_window_title(f'{current} - {title}')
        self.add(self.robot)
        
    def traj(self, q: np.array):
        """ Plot robot trajectory based on joints position

        Args:
            q (np.array): joints position
        """
        for qk in q:
            self.q = qk
            self.step()