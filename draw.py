from Models import DH_LBR_iiwa as LBR_iiwa
from CoppeliaSim.robotSimulator import RobotSimulator
import numpy as np

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, scene = 'main_scene.ttt', drawing = True, gripper = False, vision = False)

robot.Coppelia.start()

robot.Coppelia.setJointsTargetVelocity([np.pi,0,0,0,0,0,0])
start_time = robot.Coppelia.sim.getSimulationTime()
while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
    robot.Coppelia.step()

robot.Coppelia.stop()