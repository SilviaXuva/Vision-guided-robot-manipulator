import numpy as np
from Data.targets import Target

green = Target(
    x = +0.600, y = 0.1, z = 0.225, 
    rx = -np.pi, ry = 0, rz = np.pi/2, 
    gripperActuation = 'close', shape_path = './green'
)

blue = Target(
    x = +0.600, y = 0, z = 0.225, 
    rx = -np.pi, ry = 0, rz = np.pi/2, 
    gripperActuation = 'close', shape_path = './blue'
)

red = Target(
    x = +0.600, y = -0.1, z = 0.225, 
    rx = -np.pi, ry = 0, rz = np.pi/2, 
    gripperActuation = 'close', shape_path = './red'
)