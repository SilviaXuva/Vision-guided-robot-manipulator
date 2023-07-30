import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialmath import SE3
import spatialmath.base as base
import time
from tkinter import messagebox

class Env(PyPlot):  
    def __init__(self, robot) -> None:
        super().__init__()
        self.robot = robot
        self.new()
        self.controlled = list()
    
    def new(self, title = '', fig = None):
        """ Initialize new matplotlib plot

        Args:
            robot
            title (str, optional): figure window title. Defaults to ''.
        """
        limits = [-1, 1, -1, 1, 0, 1.5]
        self.launch(fig = fig, limits=limits)
        if title != '':
            current = fig.canvas.manager.get_window_title()
            fig.canvas.manager.set_window_title(f'{current} - {title}')
        self.add(self.robot)
        self.workspace = self.ax.get_figure()
        return fig
    
    def traj(self):
        self.robot.q = self.robot.getJointPosition()
        self.controlled.append(self.robot.q)
        self.step()
        
    def pose(self, T):
        """ Plot pose

        Args:
            T (SE3): pose
        """
        base.trplot(T.A, ax=self.ax, length=0.3, width=1, style='rviz')
    
    def clear(self):
        self.workspace.clf()
        self.new(fig = self.workspace)
            
    def path(self, poses: np.array, label: str = 'Cartesian path'):
        """ Plot cartesian path based on cartesian position

        Args:
            poses (np.array): end-effector poses
            label (str): path label to legend. Defaults to 'Cartesian path'.
        """
        x, y, z = np.transpose([SE3(pose).t for pose in poses])
        self.ax.plot3D(x, y, z, label=label)[0]
        self.ax.legend()
    
    def plot_ref(self, T1, traj):
        self.pose(T1)
        self.path(
            traj, 
            label = 'Reference path'
        )

    def plot_real(self, block = False):
        traj = [self.robot.fkine(q) for q in self.controlled]
        self.path(
            traj, 
            label = 'Real path'
        )
        self.pose(traj[-1])
        if block:
            messagebox.showinfo('Exit?', 'Are you sure you want to exit?', icon = 'warning')
        else:
            time.sleep(5)
        self.clear()