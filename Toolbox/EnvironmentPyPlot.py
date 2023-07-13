import matplotlib.pyplot as plt
import numpy as np
import os
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialmath import SE3
import spatialmath.base as base
from tkinter import messagebox

class Env(PyPlot):  
    def __init__(self, robot) -> None:
        super().__init__()
        self.robot = robot
    
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
        
    # def point3D(self, cart: np.array = np.array([0,0,0]), label: str = 'Target', color: str = 'red'):
    #     """ Plot 3D Point

    #     Args:
    #         cart (np.array): one cartesian point [x,y,z]. Defaults to np.array([0,0,0]).
    #         label (str): point label. Defaults to 'Target'.
    #         color (str): color of point. Defauls to 'red' 
    #     """
    #     x, y, z = [np.array([c]) for c in cart]
    #     markerline, stemlines, baseline = self.ax.stem(
    #         x, y, z, linefmt='C5-.', label=label)
    #     markerline.set_markerfacecolor('none')
    #     markerline.set_markeredgecolor(color)
    #     self.ax.legend()
    
    def pose(self, T):
        """ Plot pose

        Args:
            T (SE3): pose
        """
        base.trplot(T.A, ax=self.ax, length=0.3, width=1, style='rviz')
    
    def clear(self):
        self.workspace.clf()
        self.new(fig = self.workspace)
            
    def path(self, poses: np.array, label: str = 'Cartesian path', save_path: str = ''):
        """ Plot cartesian path based on cartesian position

        Args:
            poses (np.array): end-effector poses
            label (str): path label to legend. Defaults to 'Cartesian path'.
        """
        x, y, z = np.transpose([SE3(pose).t for pose in poses])
        self.ax.plot3D(x, y, z, label=label)[0]
        self.ax.legend()
        if save_path != '':
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            self.workspace.savefig(save_path)

    def joints(
        self,
        ref = np.array([]),
        real = np.array([]),
        qlim = True,
        save_path='',
        block=False,
        loc=None,
        grid=True,
        **kwargs,
    ):
        """Plot joints angles"""

        n = self.robot.number_joints
        
        t_ref = np.arange(0, len(ref))
        t_real = np.arange(0, len(real))
        t_limit = np.arange(0, max(t_ref[-1], t_real[-1]))
        
        fig, ax = plt.subplots()

        labels = []
        for i in range(n):
            ax = plt.subplot(n, 1, i + 1)

            if len(ref) > 0:
                plt.plot(t_ref, ref[:, i], **kwargs)
                labels += ['q_ref']
            if len(real) > 0:
                plt.plot(t_real, real[:, i], "--", **kwargs)
                labels += ['q_real']
            
            if qlim:
                plt.plot(t_limit, [self.robot.qlim[0][i] for t in t_limit], "*", **kwargs)
                plt.plot(t_limit, [self.robot.qlim[1][i] for t in t_limit], "*", **kwargs)
                labels += ['q_min', 'q_max']

            plt.grid(grid)
            ax.set_ylabel(f"Joint {i} coordinates (rad)")
            ax.set_xlim(
                min(t_ref[0], t_real[0]), 
                max(t_ref[-1], t_real[-1])
            )

        ax.legend(labels, loc=loc)
        ax.set_xlabel("Time (s)")
        
        if save_path != '':
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path)
        
        if block:
            wait(fig, save_path)
        else:
            plt.close(fig)
    
def wait(fig, save_path):
    MsgBox = messagebox.showinfo('Exit?', 'Are you sure you want to exit?', icon = 'warning')
    if MsgBox == 'ok':
        plt.close(fig)