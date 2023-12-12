from settings import Settings, Gripper, Aruco
from Data.pose import Pose
from VisionProcessing.aruco import Marker
from Simulators.CoppeliaSim import Actuation
from Helpers.measures import Real, Ref
from Data.transformations import GetDot

import numpy as np
import pandas as pd
import os

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

class Target():
    def __init__(self, name, T, GripperActuation, t, trajectoryType, trajectorySource, controller, Kp, tol) -> None:
        self.name = name
        self.T = T
        self.GripperActuation = GripperActuation
        self.t = t
        self.trajectoryType = trajectoryType
        self.trajectorySource = trajectorySource
        self.controller = controller
        self.Kp = Kp
        self.tol = tol
        self.path = fr'{Settings.executionPath}\{name}'
        self.measures = []

    def SaveData(self, robot):
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

t = np.arange(0, 10 + Settings.Ts, Settings.Ts)
Kp = np.array([8, 8, 8, 6, 6, 6])
tol = np.array([0.08, 0.1])

def GetArucoPickPlace(marker: Marker):
    if marker.T is not None:
        bin = bins[marker.color]
        alignPickPlace = [
            Target(
                name = f'{marker.color}{marker.id}\Align',
                T = Pose(
                    x = marker.T.t[0], 
                    y = marker.T.t[1], 
                    z = marker.T.t[2] + Gripper.increaseHeight, 
                    rpy = marker.T.rpy()
                )*Gripper.rotation,
                GripperActuation = Actuation(),
                t = t,
                trajectoryType = 'joint',
                trajectorySource = 'rtb',
                controller = 'rtb',
                Kp = Kp,
                tol = tol
            ),
            Target(
                name = f'{marker.color}{marker.id}\Pick',
                T = Pose(
                    x = marker.T.t[0], 
                    y = marker.T.t[1], 
                    z = marker.T.t[2] - Aruco.length/2, 
                    rpy = marker.T.rpy()
                )*Gripper.rotation,
                GripperActuation = Actuation(
                    actuation = 'close', 
                    shapePath = f'./{marker.color}{marker.id}'
                ),
                t = np.arange(0, 10 + Settings.Ts, Settings.Ts),
                trajectoryType = 'joint',
                trajectorySource = 'rtb',
                controller = 'rtb',
                Kp = Kp,
                tol = tol
            ),
            Target(
                name = f'{marker.color}{marker.id}\Place',
                T = bin,
                GripperActuation = Actuation(
                    actuation = 'open', 
                    shapePath = f'./{marker.color}{marker.id}'
                ),
                t = t,
                trajectoryType = 'joint',
                trajectorySource = 'rtb',
                controller = 'rtb',
                Kp = Kp,
                tol = tol              
            )
        ]
        return alignPickPlace
    else:
        return None