from settings import Settings
from CoppeliaSim.gripper import Actuation
from Kinematics.trajectoryPlanning import Trajectory
from Kinematics.control import Controller
from Kinematics.measures import Ref, Real, getDot, poseToCart

from spatialmath import SE3
import numpy as np
import os
import pandas as pd

class Pose:
    def __init__(self, x: float, y: float, z: float, rpy: np.ndarray = None, r_xyz: np.ndarray = None) -> None:
        self.x = x
        self.y = y
        self.z = z
        if rpy is None:
            self.rx = r_xyz[0]
            self.ry = r_xyz[1]
            self.rz = r_xyz[2]
            self.T = SE3.Trans(
                self.x, self.y, self.z
            )*SE3.Rx(self.rx)*SE3.Ry(self.ry)*SE3.Rz(self.rz)
        else:
            self.rpy = rpy
            self.T = SE3.Trans(
                self.x, self.y, self.z
            )*SE3.RPY(self.rpy) # pre defined order => zyx

            self.T = self.T*Settings.gripper_rotation
            # self.T = SE3(np.vstack([np.hstack([self.T.R@np.linalg.inv(Settings.gripper_rotation.R)@self.T.R, self.T.t.reshape(3,1)]), np.array([0,0,0,1])]))

class Target(Pose):
    def __init__(self, x: float, y: float, z: float, rpy: np.ndarray = None, r_xyz: np.ndarray = None, gripper_actuation = Actuation(), trajectory = Trajectory(), controller = Controller(), tolerance: np.ndarray = np.array([[0.1]])) -> None:
        Pose.__init__(self, x, y, z, rpy, r_xyz)
        self.Gripper_actuation = gripper_actuation
        self.Trajectory = trajectory
        self.Controller = controller
        self.tolerance = tolerance

        self.measures = []

    def log(self):
        return self.__dict__

    def saveData(self, robot):
        cart_path = fr'{self.path}\Cart Comparison'
        joint_path = fr'{self.path}\Joint Comparison'
        os.makedirs(cart_path, exist_ok=True); os.makedirs(joint_path, exist_ok=True) 
        self.Real = Real(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,0]]).to_dict(orient="list"))
        self.Ref = Ref(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,1]]).to_dict(orient="list"))
        # for m in [['q_dot', 'q'], ['q_dot_dot', 'q_dot'], ['x_dot', 'x'], ['x_dot_dot', 'x_dot']]:
        #     for type in ['Real', 'Ref']:
        #         setattr(getattr(self, type), m[0], getDot(getattr(getattr(self, type), m[1]), getattr(getattr(self, type), m[1])[0]))
        for m in [['q'], ['q_dot', 'q'], ['q_dot_dot', 'q_dot'], ['x'], ['x_dot', 'x'], ['x_dot_dot', 'x_dot']]:
            measure = m[0]
            for type in ['Real', 'Ref']:
                if len(m) == 2 and (getattr(getattr(self, type), m[0]) is None or any(elem is None for elem in getattr(getattr(self, type), m[0]))):
                    setattr(getattr(self, type), m[0], getDot(getattr(getattr(self, type), m[1]), getattr(getattr(self, type), m[1])[0]))
                setattr(getattr(self, type), f'{measure}_df', pd.DataFrame(getattr(getattr(self, type), measure), index=np.array([i*Settings.Ts for i in range(len(self.measures))])))
                if 'x' in measure:
                    getattr(getattr(self, type), f'{measure}_df').columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
                    getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{cart_path}\{measure}_{type}.csv')
                else:
                    getattr(getattr(self, type), f'{measure}_df').columns = [f'q{i}' for i in range(robot.number_joints)]
                    getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{joint_path}\{measure}_{type}.csv')

def setupTrajControl(align: Target, pick: Target, place: Target):
    align.Trajectory = Trajectory(type = 'joint', source = 'rtb', t = np.arange(0, 10 + Settings.Ts, Settings.Ts)) 
    align.Controller = Controller(type = 'cart', Kp = np.array([8, 8, 8, 10, 10, 10]))
    align.tolerance = np.array([[0.005], [0.1]])
    align.path = fr'{Settings.execution_path}\{pick.Gripper_actuation.shape_path.replace("./", "")}\Align'

    pick.Trajectory = Trajectory(type = 'cart', source = 'rtb', t = np.arange(0, 1 + Settings.Ts, Settings.Ts))
    pick.Controller = Controller(type = 'cart', Kp = np.array([8, 8, 8, 10, 10, 10]))
    pick.tolerance = np.array([[0.005], [0.1]])
    pick.path = fr'{Settings.execution_path}\{pick.Gripper_actuation.shape_path.replace("./", "")}\Pick'

    place.Trajectory = Trajectory(type = 'joint', source = 'rtb', t = np.arange(0, 10 + Settings.Ts, Settings.Ts))
    place.Controller = Controller(type = 'cart', Kp = np.array([8, 8, 8, 10, 10, 10]))
    place.tolerance = np.array([[0.005], [0.1]])
    place.path = fr'{Settings.execution_path}\{pick.Gripper_actuation.shape_path.replace("./", "")}\Place'

    return align, pick, place, 

bins = {
    "green": Target(
        x = -0.500, y = 0.600, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0],
        gripper_actuation = Actuation('open')
    ),
    "blue": Target(
        x = -0.500, y = 0, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0],
        gripper_actuation = Actuation('open')
    ),
    "red": Target(
        x = -0.500, y = -0.600, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0],
        gripper_actuation = Actuation('open')
    )
}