""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Kinematics.control import control
from Coopelia.robotSimulator import RobotSimulator
from Data.targets import pick_place_target_red as targets
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
import roboticstoolbox as rtb
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = False)

robot.Coppelia.start()

for i, target in enumerate(targets):
    
    target.pathPlanning(robot, target.T, Settings.Traj)
    
    control(robot, target)
    
    target.plot(save = True)

robot.Coppelia.stop()