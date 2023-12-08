from Kinematics import TrajectoryPlanning, isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController, Real, Ref, poseToCart
from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
from Data import plotOutputs, green
import numpy as np
from spatialmath import SE3
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = False)

robot.Coppelia.start()

target = green
if hasattr(robot.Coppelia, 'Gripper') and target.close_gripper is not None:
    robot.Coppelia.Gripper.setActuationType(target.close_gripper, shape_path = target.shape_path)
    start_time = robot.Coppelia.sim.getSimulationTime()
    while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
        robot.Coppelia.step()
        robot.Coppelia.Gripper.actuation()