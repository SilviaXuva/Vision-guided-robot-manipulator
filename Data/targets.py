from spatialmath import SE3
import spatialmath.base as base
import numpy as np
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import pandas as pd
from settings import Settings
from Kinematics.measures import Real, Ref, getDot

real_color = ('red', 'red', 'red')
ref_color = ('blue', 'blue', 'blue')

class Pose:
    def __init__(self, x, y, z, rx, ry, rz) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        
        self.T = SE3.Trans(
            self.x, self.y, self.z
        )*SE3.Rx(self.rx)*SE3.Ry(self.ry)*SE3.Rz(self.rz)

class Target(Pose):
    def __init__(self, x, y, z, rx, ry, rz, gripperActuation = None, shape_path = None) -> None:
        Pose.__init__(self, x, y, z, rx, ry, rz)
        if gripperActuation == 'close':
            self.close_gripper = True
        elif gripperActuation == 'open':
            self.close_gripper = False
        elif gripperActuation is None:
            self.close_gripper = None
        self.shape_path = shape_path if isinstance(shape_path, str) else None

        self.measures = []

    def prepare_plot(self):
        self.path = fr'{Settings.execution_path}\{str(self.shape_path).replace("./", "")}\Close gripper={str(self.close_gripper)}'
        self.cart_path = fr'{self.path}\Cart Comparison'
        self.joint_path = fr'{self.path}\Joint Comparison'
        os.makedirs(self.cart_path, exist_ok=True); os.makedirs(self.joint_path, exist_ok=True) 
        self.Real = Real(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,0]]).to_dict(orient="list"))
        self.Real.x_dot = getDot(self.Real.x, self.Real.x[0])
        self.Ref = Ref(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,1]]).to_dict(orient="list"))
        for measure in ['color', 'x', 'x_dot', 'q', 'q_dot']:
            for type in ['Real', 'Ref']:
                if measure == 'color':
                    if type == 'Real':
                        setattr(getattr(self, type), 'color', [real_color for _ in range(len(self.measures))])
                    else:
                        setattr(getattr(self, type), 'color', [ref_color for _ in range(len(self.measures))])
                else:
                    setattr(getattr(self, type), f'{measure}_df', pd.DataFrame(getattr(getattr(self, type), measure)))
                    if 'x' in measure:
                        getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{self.cart_path}\{measure}_{type}.csv', index=False)
                    else:
                        getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{self.joint_path}\{measure}_{type}.csv', index=False)

    def plot_x(self):
        ax = None
        for x, color in list(zip(self.Real.x, self.Real.color)) + list(zip(self.Ref.x, self.Ref.color)):
            T = SE3.Trans(x[:3])*SE3.Eul(x[3:]).A
            ax = base.trplot(T, ax=ax, length=0.005, width=1, axislabel=False, axissubscript=False, color=color)
        fig = ax.get_figure().savefig(fr'{self.cart_path}\cart traj-pose.png')
        plt.close(fig)

        # # for i in range(3):
        # #     pyplot = robot.plot(np.array([self.q_real[-1]]), limits=[-1, 1, -1, 1, 0, 1.5])
        # #     if i == 0 or i == 2:
        # #         base.trplot(self.T.A, ax=pyplot.ax, length=0.3, width=1, style='rviz', axislabel=False, frame='Target')
        # #         base.trplot(SE3.Trans(self.x_real[0][:3])*SE3.Eul(self.x_real[0][3:]).A, ax=pyplot.ax, length=0.1, width=1, style='rviz', labels=("X", "Y", "Z"), frame='Initial')
        # #     elif i == 1 or i == 2:
        # #         x_ref, y_ref, z_ref = np.transpose([coord[:3] for coord in self.x_ref])
        # #         pyplot.ax.plot3D(x_ref, y_ref, z_ref, color='blue')
        # #         x_real, y_real, z_real = np.transpose([coord[:3] for coord in self.x_real])
        # #         pyplot.ax.plot3D(x_real, y_real, z_real, color='red')
        # #     fig = pyplot.ax.get_figure().savefig(fr'{path}\Trajectory Comparison_{i+1}.png')
        # #     plt.close(fig)

        for measure in ['x', 'x_dot']:
            columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
            for plot_columns in [['x', 'y', 'z'], ['rx', 'ry', 'rz']]:
                fig, ax = plt.subplots()
                fig.set_figheight(5)
                fig.set_figwidth(15)
                for i, column in enumerate(plot_columns): 
                    ax = plt.subplot(1, 3, i + 1)
                    for type in ['Real', 'Ref']:
                        getattr(getattr(self, type), f'{measure}_df').columns = [f'{c}_{type}' for c in columns]
                        getattr(getattr(self, type), f'{measure}_df')[f'{column}_{type}'].plot(ax=ax, legend=True)
                        ax.title.set_text(column)
                fig.savefig(fr'{self.cart_path}\{measure}_{",".join(plot_columns)}.png')

    def plot_q(self, robot):    
        for measure in ['q', 'q_dot']:
            plot_columns = [f'q{i}' for i in range(robot.number_joints)]
            fig, ax = plt.subplots()
            fig.set_figheight(30)
            fig.set_figwidth(15)            
            for i, column in enumerate(plot_columns):
                for type in ['Real', 'Ref']:
                    ax = plt.subplot(robot.number_joints, 1, i + 1)
                    getattr(getattr(self, type), f'{measure}_df').columns = [f'q{c}_{type}' for c in range(robot.number_joints)]
                    getattr(getattr(self, type), f'{measure}_df')[f'{column}_{type}'].plot(ax=ax, legend=True)
                    ax.title.set_text(column)
            fig.savefig(fr'{self.joint_path}\{measure}.png')

bins = {
    "green": Target(
        -0.500, 0.600, 0.5, 
        -np.pi/2, 0, 0, 
        'open', './green'
    ),
    "blue": Target(
        -0.500, 0, 0.5, 
        -np.pi/2, 0, np.pi/2, 
        'open', './blue'
    ),
    "red": Target(
        -0.500, -0.600, 0.5, 
        np.pi/2, 0, 0, 
        'open', './red'
    )
}
