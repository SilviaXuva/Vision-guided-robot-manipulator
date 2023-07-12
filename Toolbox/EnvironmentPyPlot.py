import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialmath import SE3

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
        return fig
        
    def point3D(self, cart: np.array = np.array([0,0,0]), label: str = 'Target', color: str = 'red'):
        """ Plot 3D Point

        Args:
            cart (np.array): one cartesian point [x,y,z]. Defaults to np.array([0,0,0]).
            label (str): point label to legend. Defaults to 'Target'.
        """
        x, y, z = [np.array([c]) for c in cart]
        pointLine = markerline, stemlines, baseline = self.ax.stem(
            x, y, z, linefmt='C5-.', label=label)
        markerline.set_markerfacecolor('none')
        markerline.set_markeredgecolor(color)
        self.ax.legend()
        return pointLine
    
    def clear(self, lines):
        for line in lines: 
            line.remove()
            
    def path(self, poses: np.array, label: str = 'Cartesian path'):
        """ Plot cartesian path based on cartesian position

        Args:
            poses (np.array): end-effector poses
            label (str): path label to legend. Defaults to 'Cartesian path'.
        """
        x, y, z = np.transpose([SE3(pose).t for pose in poses])
        pathLine = self.ax.plot3D(x, y, z, label=label)[0]
        self.ax.legend()
        return pathLine