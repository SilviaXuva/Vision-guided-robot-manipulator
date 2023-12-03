from spatialmath import SE3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import pandas as pd
from settings import Settings

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
        self.q_ref = list()
        self.q_real = list()

    def plot(
        self,
        robot,
        q_ref = True,
        q_real = True,
        qlim = True,
        save = False,
        block = False,
        loc = None,
        grid=True,
        **kwargs,
    ):    
        t_ref = np.arange(0, len(self.q_ref))
        t_real = np.arange(0, len(self.q_real))
        t_limit = np.arange(0, max(t_ref[-1], t_real[-1]))
        
        fig, ax = plt.subplots()

        labels = []
        for i in range(robot.number_joints):
            ax = plt.subplot(robot.number_joints, 1, i + 1)

            if q_ref:
                plt.plot(t_ref, np.array(self.q_ref)[:, i], **kwargs)
                labels += ['q_ref']
            if q_real:
                plt.plot(t_real, np.array(self.q_real)[:, i], "--", **kwargs)
                labels += ['q_real']
            
            if qlim:
                plt.plot(t_limit, [robot.qlim[0][i] for t in t_limit], "*", **kwargs)
                plt.plot(t_limit, [robot.qlim[1][i] for t in t_limit], "*", **kwargs)
                labels += ['q_min', 'q_max']

            plt.grid(grid)
            # ax.set_ylabel(f"Joint {i} coordinates (rad)")
            ax.set_xlim(
                min(t_ref[0], t_real[0]), 
                max(t_ref[-1], t_real[-1])
            )

        ax.legend(labels, loc=loc)
        ax.set_xlabel("Time (s)")
        
        path = fr'{Settings.execution_path}\{self.shape_path.replace("./", "")}\Close gripper={self.close_gripper}\{Settings.start_time}'
        os.makedirs(path, exist_ok=True)
        if save:
            fig.savefig(fr'{path}\Comparison.png')
        if block:
            plt.show()
        
        q_ref_df = pd.DataFrame(self.q_ref)
        q_ref_df.to_csv(fr'{path}\q_ref.csv', index=False)
        q_df = pd.DataFrame(self.q_real)
        q_df.to_csv(fr'{path}\q.csv', index=False)        

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

initial = Target(
    -0.00680924483, -2.143585958e-06, 1.409424819, 
    1.331635568e-07, 3.670433642e-05, 0.01648125425, 
    None, None
)