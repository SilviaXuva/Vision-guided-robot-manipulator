from Kinematics.trajectory import quinticEndEffectorTraj, quinticJointTraj
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import roboticstoolbox as rtb
from settings import Settings
from spatialmath import SE3

class PathPlanning:
    def __init__(self, robot, T1, trajectory) -> None:
        self.robot_number_joints = robot.number_joints
        self.q0 = robot.Coppelia.getJointsPosition()
        self.T0 = robot.fkine(self.q0)
        self.T1 = T1
        self.q1 = robot.ikine_LMS(self.T1).q

        self.q_ref = list()
        if trajectory.type == 'cart':
            if trajectory.source == 'rtb':
                self.ctraj = rtb.ctraj(self.T0, self.T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
                self.ref = [SE3(pose) for pose in self.ctraj.A]  # Transform each pose into SE3
            elif trajectory.source == 'custom':
                self.ctraj = quinticEndEffectorTraj(self.T0, self.T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
                self.ref = [SE3.Trans(x[:3])*SE3.Eul(x[3:]) for x in self.ctraj.x]  # Transform each pose into SE3
            for pose in self.ref:
                robot.Coppelia.step()
                self.q_ref.append(robot.ikine_LMS(pose).q)
        elif trajectory.type == 'joint':
            if trajectory.source == 'rtb':
                self.jtraj = rtb.jtraj(self.q0, self.q1, Settings.t)  # Calculate reference joints trajectory
                self.ref = [SE3(robot.fkine(q)) for q in self.jtraj.q]  # Transform each joint position into SE3 through forward kinematics
            if trajectory.source == 'custom':
                self.jtraj = quinticJointTraj(self.q0, self.q1, Settings.t)  # Calculate reference joints trajectory
                self.ref = [SE3(robot.fkine(q)) for q in self.jtraj.q]  # Transform each joint position into SE3 through forward kinematics
            for q in self.jtraj.q:
                robot.Coppelia.step()
                self.q_ref.append(q)
        self.q = list()
        self.q_lim = robot.qlim

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
        t_ref = np.arange(0, len(self.q_ref))
        t_real = np.arange(0, len(self.q))
        t_limit = np.arange(0, max(t_ref[-1], t_real[-1]))
        
        fig, ax = plt.subplots()

        labels = []
        for i in range(self.robot_number_joints):
            ax = plt.subplot(self.robot_number_joints, 1, i + 1)

            if q_ref:
                plt.plot(t_ref, np.array(self.q_ref)[:, i], **kwargs)
                labels += ['q_ref']
            if q:
                plt.plot(t_real, np.array(self.q)[:, i], "--", **kwargs)
                labels += ['q_real']
            
            if qlim:
                plt.plot(t_limit, [self.q_lim[0][i] for t in t_limit], "*", **kwargs)
                plt.plot(t_limit, [self.q_lim[1][i] for t in t_limit], "*", **kwargs)
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
        q_df = pd.DataFrame(self.q)
        q_df.to_csv(fr'{path}\q.csv', index=False)
        