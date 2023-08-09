from datetime import datetime
import numpy as np

class Pose():
    def __init__(self, x, y, z, rx, ry, rz, gripperActuation=None, shapePath=None) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.gripperClose = True if gripperActuation == 'close' else False
        self.shapePath = shapePath.title() if isinstance(shapePath, str) else None
        
red = [
    Pose(
        0.75, -0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './Red'
    ),
    Pose(
        -7.2168e-03, -6.3136e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './Red'
    )
]

green = [
    Pose(
        0.75, 0.1, 0.43, -np.pi, 0, np.pi/2, 'close', './Green'
    ),
    Pose(
        -6.3857e-01, -9.7901e-05, +7.7870e-01, np.pi/2, 0, 0, 'open', './Green'
    )
]

blue = [
    Pose(
        0.75, 0, 0.43, -np.pi, 0,  np.pi/2, 'close', './Blue'
    ),
    Pose(
        -4.5363e-01, -4.4648e-01, +7.7870e-01, np.pi/2, 0, 0, 'open', './Blue'
    )
]

initial = Pose(
    0, 0, 1.40836875e+00, 0, 0, 0
)

pick_place_target_red_with_initial = [
    red[0],
    initial,
    red[1]
]

pick_place_target_red_without_initial = [
    red[0],
    red[1]
]

all_pick_place_with_initial_between = [
    red[0],
    initial,
    red[1],
    initial,
    green[0],
    initial,
    green[1],
    initial,
    blue[0],
    initial,
    blue[1]
]

all_pick_place_without_initial = [
    red[0],
    red[1],
    green[0],
    green[1],
    blue[0],
    blue[1],
]