""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController
from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
from settings import Settings

from Kinematics.trajectoryPlanning import TrajectoryPlanning
from Data.targets import bins, initial
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
    # if Settings.Trajectory.type == 'cart':
    #     for pose in traj:
    #         q_ref = robot.ikine_LMS(pose).q
    # elif Settings.Trajectory.type == 'joint':
    #     for q in traj:
    #         q_ref = q
    x0 = np.block([target.T0.t, target.T0.eul()])  # Initial end-effector position
    for pose in target.traj:
        q_ref = robot.ikine_LMS(pose).q
        q = robot.Coppelia.getJointsPosition()
        if Settings.plot:
            target.q_ref.append(robot.ikine_LMS(pose).q)
            target.q_real.append(q)

        if isClose(robot, target.T, q):
            print(f'End-Effector is close to target {i+1}')
            break

        T_ref = pose
        x_ref = np.block([T_ref.t, T_ref.eul()])
        x_dot_ref = (x_ref - x0)/Settings.Ts
        x0 = x_ref

        if Settings.Controller.type == 'cart':
            q_control, q_dot_control = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
        elif Settings.Controller.type == 'joint':
            q_control, q_dot_control = jointSpaceController(robot, q_ref, x_dot_ref, q)
        elif Settings.Controller.type is None:
            q_dot_control = inverseDifferentialKinematics(robot, q, x_dot_ref)

        robot.Coppelia.setJointsTargetVelocity(q_dot_control)
        robot.Coppelia.step()

    if Settings.plot:
        target.plot(robot, save = True)
    robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
    robot.Coppelia.step()
        
robot.Coppelia.stop()