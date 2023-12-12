from Data.pose import Pose
from VisionProcessing.aruco import Marker

import numpy as np

deg = np.pi/180
green = Marker(None, 1, 'green', T = Pose(
    x = 0.600, y = 0.15, z = 0.525,
    r_xyz = [0, 0, 0]
))

blue = Marker(None, 2, 'blue', T = Pose(
    x = 0.600, y = 0, z = 0.525, 
    r_xyz = [0, 0, 0]
))

red = Marker(None, 3, 'red', T = Pose(
    x = 0.600, y = -0.15, z = 0.525,
    r_xyz = [0, 0, 60*deg]
))