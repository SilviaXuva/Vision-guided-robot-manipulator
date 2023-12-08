import numpy as np
from Data.targets import Target
from settings import Settings

green = Target(
    x = +0.600, y = 0.2, z = 0.225, 
    rpy = None, # [-np.pi,  0, -np.pi/2]
    r_xyz = [-np.pi, 0, np.pi/2], 
    gripperActuation = 'close', shape_path = './green',
)


blue = Target(
    x = +0.600, y = 0, z = 0.225, 
    rpy = None, # [-np.pi,  0, -np.pi/2]
    r_xyz = [-np.pi, 0, np.pi/2], 
    gripperActuation = 'close', shape_path = './blue'
)

red = Target(
    x = +0.600, y = -0.2, z = 0.225, 
    rpy = None, # [-np.pi,  0, -np.pi/2]
    r_xyz = [-np.pi, 0, np.pi/2],  
    gripperActuation = 'close', shape_path = './red'
)