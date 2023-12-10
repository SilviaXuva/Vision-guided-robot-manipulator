from Data.targets import Target

import numpy as np

deg = np.pi/180
green = Target(
    x = +0.600, y = 0.2, z = 0.225,
    r_xyz = [0, 0, 0]
)

blue = Target(
    x = +0.600, y = 0, z = 0.225, 
    r_xyz = [0, 0, 0]
)

red = Target(
    x = +0.600, y = -0.2, z = 0.225,
    r_xyz = [0, 0, 60*deg]
)