from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = True)

robot.Coppelia.start()

while True:
    robot.Coppelia.step()
    pick_place_targets = robot.Coppelia.Vision.getTargets()
    if len(pick_place_targets) > 0:
        pass

robot.Coppelia.stop()