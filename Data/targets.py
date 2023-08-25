from spatialmath import SE3
import numpy as np
from Data.pose import Pose
from Data.trajectory import Trajectory

class Target(Pose, Trajectory):
    def __init__(self, x, y, z, rx, ry, rz, gripperActuation = None, shape_path = None) -> None:
        Pose.__init__(self, x, y, z, rx, ry, rz)
        self.close_gripper = True if gripperActuation == 'close' else False
        self.shape_path = shape_path.title() if isinstance(shape_path, str) else None
    
    def getTrajectory(self, robot, T1, type):
        Trajectory.__init__(self, robot, T1, type)

red = [
    Target(
        0.75, -0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './Red'
    ),
    Target(
        -7.2168e-03, -6.3136e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './Red'
    )
]

green = [
    Target(
        0.75, 0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './Green'
    ),
    Target(
        -6.3857e-01, -9.7901e-05, +7.7870e-01, np.pi/2, 0, 0, 'open', './Green'
    )
]

blue = [
    Target(
        0.75, 0, 0.43, -np.pi, 0,  np.pi/2, 'close', './Blue'
    ),
    Target(
        -4.5363e-01, -4.4648e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './Blue'
    )
]

pick_place_target_red = [
    red[0],
    red[1]
]

all_pick_place = [
    red[0],
    red[1],
    green[0],
    green[1],
    blue[0],
    blue[1],
]