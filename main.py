from settings import Settings
from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController
from Kinematics.trajectoryPlanning import TrajectoryPlanning
from Kinematics.measures import Real, Ref, poseToCart
from CoppeliaSim.robotSimulator import RobotSimulator
from Models.DH_LBR_iiwa import LBR_iiwa
from Data.plotOutputs import plotOutputs
from Data.targets import setupTrajControl

from spatialmath import SE3
import numpy as np 

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = True)

robot.Coppelia.start()

while len(robot.Coppelia.Vision.aruco_markers) > 0:
    robot.Coppelia.step()
    robot.Coppelia.Vision.estimateArucoPose()
    pick_place = align, pick, place = robot.Coppelia.Vision.getTarget()

    if np.any(pick_place):
        align, pick, place = setupTrajControl(align, pick, place)

        for i, target in enumerate([align, pick, place]):
            robot.Coppelia.step()
            target.q0 = robot.Coppelia.getJointsPosition()
            target.T0 = robot.fkine(target.q0)

            target.traj = TrajectoryPlanning(robot, target)

            for j, [T_ref, q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref] in enumerate(zip(target.traj.T, target.traj.q, target.traj.q_dot, target.traj.q_dot_dot, target.traj.x, target.traj.x_dot, target.traj.x_dot_dot)):
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

                robot.Coppelia.setJointsTargetVelocity(q_dot_control)
                robot.Coppelia.step(T_ref.t)
                target.measures.append([
                    Real(robot.fkine(q_control), q_control, q_dot_control, None, poseToCart(robot.fkine(q_control)), None, None),
                    Ref(T_ref, q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref)
                ])

            target.saveData(robot)
            
            robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
            robot.Coppelia.step()

            if hasattr(robot.Coppelia, 'Gripper') and target.Gripper_actuation.close is not None:
                success = robot.Coppelia.Gripper.setupActuation(target.Gripper_actuation)
                if success:
                    start_time = robot.Coppelia.sim.getSimulationTime()
                    while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
                        robot.Coppelia.step()
                        robot.Coppelia.Gripper.actuation()
            else:
                success = True
        
            if not success:
                break

robot.Coppelia.stop()

if Settings.plot:
    plotOutputs(robot.number_joints, folder = Settings.execution_path)