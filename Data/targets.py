from spatialmath import SE3
import numpy as np
import os
import pandas as pd
from settings import Settings
from Kinematics.measures import Real, Ref, getDot

class Pose:
    def __init__(self, x, y, z, rpy = None, r_xyz = None) -> None:
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
            )*SE3.RPY(self.rpy)
        
        self.T = self.T*Settings.gripper_rotation

class Target(Pose):
    def __init__(self, x, y, z, rpy = None, r_xyz = None, gripperActuation = None, shape_path = None) -> None:
        Pose.__init__(self, x, y, z, rpy, r_xyz)
        if gripperActuation == 'close':
            self.close_gripper = True
        elif gripperActuation == 'open':
            self.close_gripper = False
        elif gripperActuation is None:
            self.close_gripper = None
        self.shape_path = shape_path if isinstance(shape_path, str) else None

        self.measures = []

    def saveData(self, robot):
        path = fr'{Settings.execution_path}\{str(self.shape_path).replace("./", "")}\Close gripper={str(self.close_gripper)}'
        cart_path = fr'{path}\Cart Comparison'
        joint_path = fr'{path}\Joint Comparison'
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

bins = {
    "green": Target(
        x = -0.500, y = 0.600, z = 0.75, 
        rpy = [7.49879891e-33,  7.49879891e-33, -3.14159265e+00], 
        r_xyz = None, 
        gripperActuation = 'open', 
    ),
    "blue": Target(
        x = -0.500, y = 0, z = 0.75, 
        rpy = [7.49879891e-33,  7.49879891e-33, -3.14159265e+00], 
        r_xyz = None, 
        gripperActuation = 'open',
    ),
    "red": Target(
        x = -0.500, y = -0.600, z = 0.75, 
        rpy = [7.49879891e-33,  7.49879891e-33, -3.14159265e+00], 
        r_xyz = None, 
        gripperActuation = 'open', 
    )
}
