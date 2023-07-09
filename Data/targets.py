import numpy as np

class Pose():
    def __init__(self, x, y, z, rx, ry, rz, gripperActuation=None, object=None) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.gripperActuation = gripperActuation
        self.object = object
        
red = [
    Pose(
        0.75, -0.1, 0.43, -np.pi, 0, np.pi/2, 'close', 'red'
    ),
    Pose(
        -7.2168e-03, -6.3136e-01, +7.7870e-01, np.pi/2, 0, 0, 'open'
    )
]

green = [
    Pose(
        0.75, 0.1, 0.43, -np.pi, 0, np.pi/2, 'close', 'green'
    ),
    Pose(
        -6.3857e-01, -9.7901e-05, +7.7870e-01, np.pi/2, 0, 0, 'open'
    )
]

blue = [
    Pose(
        0.75, 0, 0.43, -np.pi, 0,  np.pi/2, 'close', 'blue'
    ),
    Pose(
        -4.5363e-01, -4.4648e-01, +7.7870e-01, np.pi/2, 0, 0, 'open'
    )
]

initial = Pose(
    0, 0, 1.40836875e+00, 0, 0, 0
)

targets1 = [
    red[0],
    initial,
    red[1]
]

targets = [
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