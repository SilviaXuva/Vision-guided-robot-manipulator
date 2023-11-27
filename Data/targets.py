from spatialmath import SE3
import numpy as np
from Data.pathPlanning import PathPlanning

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

class Target(Pose, PathPlanning):
    def __init__(self, x, y, z, rx, ry, rz, gripperActuation = None, shape_path = None) -> None:
        Pose.__init__(self, x, y, z, rx, ry, rz)
        if gripperActuation == 'close':
            self.close_gripper = True
        elif gripperActuation == 'open':
            self.close_gripper = False
        elif gripperActuation is None:
            self.close_gripper = None
        self.shape_path = shape_path if isinstance(shape_path, str) else None
    
    def pathPlanning(self, robot, T1, trajectory):
        PathPlanning.__init__(self, robot, T1, trajectory)

red = [
    Target(
        0.75, -0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './red'
    ),
    Target(
        -7.2168e-03, -6.3136e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './red'
    )
]

green = [
    Target(
        0.75, 0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './green'
    ),
    Target(
        -6.3857e-01, -9.7901e-05, +7.7870e-01, np.pi/2, 0, 0, 'open', './green'
    )
]

blue = [
    Target(
        0.75, 0, 0.43, -np.pi, 0,  np.pi/2, 'close', './blue'
    ),
    Target(
        -4.5363e-01, -4.4648e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './blue'
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

bins = {
    "red": Target(
        -0.500, -0.600, 0.5, np.pi/2, 0, 0, 'open', './red'
    ),
    "green": Target(
        -0.500, 0.600, 0.5, -np.pi/2, 0, 0, 'open', './green'
    ),
    "blue": Target(
        -0.500, 0, 0.5, -np.pi, 0, np.pi/2, 'open', './blue'
    )
}

initial = Target(
    -0.00680924483, -2.143585958e-06, 1.409424819, 1.331635568e-07, 3.670433642e-05, 0.01648125425, None, None
)