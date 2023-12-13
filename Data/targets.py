from Data.pose import Pose
from Data.transformations import GetDot
from Helpers.measures import Real, Ref
from settings import Settings, Gripper, Aruco
from Simulators.CoppeliaSim import Actuation
from VisionProcessing.aruco import Marker

import numpy as np
import os
import pandas as pd
from roboticstoolbox import DHRobot, ERobot
from spatialmath import SE3

class Target():
    def __init__(self, 
            name: str, 
            T: SE3, 
            GripperActuation: Actuation, 
            t: np.ndarray, 
            trajectoryType: str, trajectorySource: str, 
            controller: str, Kp: np.ndarray, 
            tol: np.ndarray
        ):
        self.name = name
        self.T = T
        self.GripperActuation = GripperActuation
        self.t = t
        self.trajectoryType = trajectoryType; self.trajectorySource = trajectorySource
        self.controller = controller; self.Kp = Kp
        self.tol = tol
        self.path = fr'{Settings.executionPath}\{name}'; self.measures = []

    def SaveData(self, robot: DHRobot|ERobot):
        cartPath = fr'{self.path}\Cart Comparison'
        jointPath = fr'{self.path}\Joint Comparison'
        os.makedirs(cartPath, exist_ok=True); os.makedirs(jointPath, exist_ok=True) 
        self.Real = Real(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,0]]).to_dict(orient="list"))
        self.Ref = Ref(**pd.DataFrame([m.__dict__ for m in np.array(self.measures)[:,1]]).to_dict(orient="list"))
        for m in [['q'], ['qDot', 'q'], ['qDotDot', 'qDot'], ['x'], ['xDot', 'x'], ['xDotDot', 'xDot']]:
            measure = m[0]
            for type in ['Real', 'Ref']:
                if len(m) == 2 and (getattr(getattr(self, type), m[0]) is None or any(elem is None for elem in getattr(getattr(self, type), m[0]))):
                    setattr(getattr(self, type), m[0], GetDot(getattr(getattr(self, type), m[1]), getattr(getattr(self, type), m[1])[0]))
                setattr(getattr(self, type), f'{measure}_df', pd.DataFrame(getattr(getattr(self, type), measure), index=np.array([i*Settings.Ts for i in range(len(self.measures))])))
                if 'x' in measure:
                    getattr(getattr(self, type), f'{measure}_df').columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
                    getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{cartPath}\{measure}_{type}.csv')
                else:
                    getattr(getattr(self, type), f'{measure}_df').columns = [f'q{i}' for i in range(robot.n)]
                    getattr(getattr(self, type),f'{measure}_df').to_csv(fr'{jointPath}\{measure}_{type}.csv')

bins = {
    "green": Pose(
        x = -0.500, y = 0.600, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0]
    ),
    "blue": Pose(
        x = -0.500, y = 0, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0]
    ),
    "red": Pose(
        x = -0.500, y = -0.600, z = 0.75, 
        r_xyz = [-np.pi/2,  -np.pi/2, 0]
    )
}

def GetArucoPickPlace(robot: DHRobot|ERobot, marker: Marker):
    if marker.T is not None:
        bin = bins[marker.color]
        pickPlace = [
            Target(
                name = f'{marker.color}{marker.id}\Align',
                T = Pose(
                    x = marker.T.t[0], 
                    y = marker.T.t[1], 
                    z = marker.T.t[2] + Gripper.increaseHeight, 
                    rpy = marker.T.rpy()
                )*Gripper.rotation,
                GripperActuation = Actuation(),
                t = Settings.t,
                trajectoryType = Settings.trajectoryType, trajectorySource = Settings.trajectorySource,
                controller = Settings.controller, Kp = Settings.Kp,
                tol = Settings.tol
            ),
            Target(
                name = f'{marker.color}{marker.id}\Pick',
                T = Pose(
                    x = marker.T.t[0], 
                    y = marker.T.t[1], 
                    z = marker.T.t[2] - Aruco.length/4, 
                    rpy = marker.T.rpy()
                )*Gripper.rotation,
                GripperActuation = Actuation(
                    actuation = 'close', 
                    shapePath = f'./{marker.color}{marker.id}'
                ),
                t = np.arange(0, 2 + Settings.Ts, Settings.Ts),
                trajectoryType = Settings.trajectoryType, trajectorySource = Settings.trajectorySource,
                controller = Settings.controller, Kp = Settings.Kp,
                tol = Settings.tol
            ),
            Target(
                name = f'{marker.color}{marker.id}\Place',
                T = bin,
                GripperActuation = Actuation(
                    actuation = 'open', 
                    shapePath = f'./{marker.color}{marker.id}'
                ),
                t = Settings.t,
                trajectoryType = Settings.trajectoryType, trajectorySource = Settings.trajectorySource,
                controller = Settings.controller, Kp = Settings.Kp,
                tol = Settings.tol
            ),
            Target(
                name = f'{marker.color}{marker.id}\Initial',
                T = robot.Tz,
                GripperActuation = Actuation(),
                t = Settings.t,
                trajectoryType = Settings.trajectoryType, trajectorySource = Settings.trajectorySource,
                controller = Settings.controller, Kp = Settings.Kp,
                tol = Settings.tol
            )
        ]
        return pickPlace
    else:
        return None