import matplotlib.pyplot as plt
import numpy as np
import os
import roboticstoolbox as rtb
from settings import Settings
from spatialmath import SE3

class Trajectory:
    def __init__(self, robot, T1, type) -> None:
        self.robot = robot
        
        self.q0 = robot.Coppelia.getJointsPosition()
        self.T0 = robot.fkine(self.q0)
        self.T1 = T1
        self.q1 = robot.ikine_LMS(self.T1).q
        if type == 'cart':
            self.ctraj = rtb.ctraj(self.T0, self.T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
            self.ref = [SE3(pose) for pose in self.ctraj.A]  # Transform each pose into SE3]
            self.q_ref = [robot.ikine_LMS(pose).q for pose in self.ctraj.A]
        elif type == 'joint':
            self.jtraj = rtb.jtraj(self.q0, self.q1, Settings.t)  # Calculate reference joints trajectory
            self.ref = [SE3(robot.fkine(q)) for q in self.jtraj.q]  # Transform each joint position into SE3 through forward kinematics
            self.q_ref = [q for q in self.jtraj.q]
        self.q = list()
        
    def plot(self, q_ref = True, q = True):
        if q_ref:
            rtb.xplot(np.array(self.q_ref))
        if q:
            rtb.xplot(np.array(self.q))

    def plot(
        self,
        q_ref = True,
        q = True,
        qlim = True,
        save = False,
        block = False,
        loc = None,
        grid=True,
        **kwargs,
    ):
        n = self.robot.number_joints
        
        t_ref = np.arange(0, len(self.q_ref))
        t_real = np.arange(0, len(self.q))
        t_limit = np.arange(0, max(t_ref[-1], t_real[-1]))
        
        fig, ax = plt.subplots()

        labels = []
        for i in range(n):
            ax = plt.subplot(n, 1, i + 1)

            if q_ref:
                plt.plot(t_ref, np.array(self.q_ref)[:, i], **kwargs)
                labels += ['q_ref']
            if q:
                plt.plot(t_real, np.array(self.q)[:, i], "--", **kwargs)
                labels += ['q_real']
            
            if qlim:
                plt.plot(t_limit, [self.robot.qlim[0][i] for t in t_limit], "*", **kwargs)
                plt.plot(t_limit, [self.robot.qlim[1][i] for t in t_limit], "*", **kwargs)
                labels += ['q_min', 'q_max']

            plt.grid(grid)
            # ax.set_ylabel(f"Joint {i} coordinates (rad)")
            ax.set_xlim(
                min(t_ref[0], t_real[0]), 
                max(t_ref[-1], t_real[-1])
            )

        ax.legend(labels, loc=loc)
        ax.set_xlabel("Time (s)")
        
        if save:
            path = fr'{Settings.output_path}\{self.shapePath.replace("./", "")}\Close gripper={self.closeGripper}.png'
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fig.savefig(path)
        if block:
            plt.show()
