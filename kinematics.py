from settings import Settings
from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController, Controller
from Kinematics.trajectoryPlanning import Trajectory, TrajectoryPlanning
from Kinematics.measures import Real, Ref, poseToCart
from CoppeliaSim.robotSimulator import RobotSimulator
from CoppeliaSim.gripper import Actuation
from Models.DH_LBR_iiwa import LBR_iiwa
from Data.plotOutputs import plotOutputs

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb

from Data.targets import bins
from Data.openLoop import green, blue, red

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, scene = 'main_scene.ttt', drawing = True, gripper = False, vision = False)

robot.Coppelia.start()

green.Gripper_actuation = Actuation(actuation = 'close', shape_path = './green1')
green.Trajectory = Trajectory(type = 'joint', source = 'rtb', t = np.arange(0, 10+Settings.Ts, Settings.Ts))
green.Controller = Controller(type = 'cart', Kp = np.array([8, 8, 8, 6, 6, 6]))
green.tolerance = np.array([[0.01]])

targets = [green]
for i, target in enumerate(targets):
    robot.Coppelia.step()
    target.q0 = robot.Coppelia.getJointsPosition()
    target.T0 = robot.fkine(target.q0)

    target.traj = TrajectoryPlanning(robot, target)

    for j, [T_ref, q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref] in enumerate(zip(target.traj.T, target.traj.q, target.traj.q_dot, target.traj.q_dot_dot, target.traj.x, target.traj.x_dot, target.traj.x_dot_dot)):
        if j >= 190:
            pass
        q = robot.Coppelia.getJointsPosition()
        if isClose(robot, target.T, q, target.tolerance):
            Settings.log(f'End-Effector is close to target {i+1}')
            break

        if q_ref is None:
            q_ref = robot.ikine_LMS(SE3.Trans(x_ref[:3])*SE3.Eul(x_ref[3:])).q

        if target.Controller.type == 'cart':
            q_control, q_dot_control = cartesianSpaceController(robot, target.Controller.Kp, q, T_ref, None)
        elif target.Controller.type == 'joint':
            q_control, q_dot_control = jointSpaceController(robot, target.Controller.Kp, q_ref, x_dot_ref, q)
        elif target.Controller.type is None:
            q_dot_control = inverseDifferentialKinematics(robot, q, control_signal = x_dot_ref)
        elif target.Controller.type == 'servo':
            v, arrived = rtb.p_servo(robot.fkine(q), T_ref, target.Controller.Kp)
            q_dot_control = inverseDifferentialKinematics(robot, q, control_signal = v)
            q_control = q + q_dot_control*Settings.Ts

        robot.Coppelia.setJointsTargetVelocity(q_dot_control)
        robot.Coppelia.step(T_ref.t)
        target.measures.append([
            Real(robot.fkine(q_control), q_control, q_dot_control, None, poseToCart(robot.fkine(q_control)), None, None),
            Ref(T_ref, q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref)
        ])

    target.saveData(robot)
    
    robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
    robot.Coppelia.step()

robot.Coppelia.stop()

if Settings.plot:
    plotOutputs(robot.number_joints, folder = Settings.execution_path)