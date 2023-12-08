from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController
from Kinematics.trajectoryPlanning import TrajectoryPlanning
from Kinematics.measures import Real, Ref, poseToCart
from CoppeliaSim.robotSimulator import RobotSimulator
from Data.plotOutputs import plotOutputs
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
from spatialmath import SE3
from settings import Settings

from Data.targets import bins
from Data.ma import green, blue, red

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, scene = 'main_scene.ttt', drawing = True, gripper = False, vision = False)

robot.Coppelia.start()

targets = [green]
for i, target in enumerate(targets):
    robot.Coppelia.step()
    target.q0 = robot.Coppelia.getJointsPosition()
    target.T0 = robot.fkine(target.q0)

    target.traj = TrajectoryPlanning(robot, target.q0, target.T, Settings.Trajectory)

    for q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref in zip(target.traj.q, target.traj.q_dot, target.traj.q_dot_dot, target.traj.x, target.traj.x_dot, target.traj.x_dot_dot):
        q = robot.Coppelia.getJointsPosition()
        if isClose(robot, target.T, q):
            print(f'End-Effector is close to target {i+1}')
            break

        if q_ref is None:
            q_ref = robot.ikine_LMS(SE3.Trans(x_ref[:3])*SE3.Eul(x_ref[3:])).q

        if Settings.Controller.type == 'cart':
            q_control, q_dot_control = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
        elif Settings.Controller.type == 'joint':
            q_control, q_dot_control = jointSpaceController(robot, q_ref, x_dot_ref, q)
        elif Settings.Controller.type is None:
            q_dot_control = inverseDifferentialKinematics(robot, q, x_dot_ref)
        robot.Coppelia.setJointsTargetVelocity(q_dot_control)
        robot.Coppelia.step(x_ref[:3])
        target.measures.append([
            Real(q_control, q_dot_control, None, poseToCart(robot.fkine(q_control)), None, None),
            Ref(q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref)
        ])

    target.saveData(robot)
    
    robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
    robot.Coppelia.step()

robot.Coppelia.stop()

if Settings.plot:
    plotOutputs(robot.number_joints, folder = Settings.execution_path)