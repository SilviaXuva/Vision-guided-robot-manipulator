from Data.pose import Pose
from VisionProcessing.aruco import Marker

import numpy as np

deg = np.pi/180
green = Marker(None, id = 1, color = 'green', T = Pose(
    x = 0.600, y = 0.15, z = 0.250,
    r_xyz = [0, 0, 100*deg]
))

blue = Marker(None, id = 2, color = 'blue', T = Pose(
    x = 0.600, y = 0, z = 0.250, 
    r_xyz = [0, 0, -125*deg]
))

red = Marker(None, id = 3, color = 'red', T = Pose(
    x = 0.600, y = -0.15, z = 0.250,
    r_xyz = [0, 0, 60*deg]
))
