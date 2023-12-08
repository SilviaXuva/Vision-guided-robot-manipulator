from Kinematics import TrajectoryPlanning, isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController, Real, Ref, poseToCart
from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
from Data import plotOutputs
import numpy as np
from spatialmath import SE3
from settings import Settings, Trajectory, Controller
from Kinematics import getDot

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = True)

robot.Coppelia.start()

while len(robot.Coppelia.Vision.aruco_markers) > 0:
    robot.Coppelia.step()
    robot.Coppelia.Vision.estimateArucoPose()
    align_target, pick_target, place_target = robot.Coppelia.Vision.getTarget(robot)

    for i, [t, target] in enumerate([align_target, pick_target, place_target]):
        robot.Coppelia.step()
        q_ref0 = target.q0 = robot.Coppelia.getJointsPosition()
        target.T0 = robot.fkine(target.q0)

        if i == 0:
            control = Settings.Controller
            traj = Settings.Trajectory
            tol_trans = 0.05; tol_rot = 0.2
        elif i == 1:
            control = Controller('cart', [8,6])
            traj = Settings.Trajectory
            tol_trans = Settings.Tolerance.tol_trans; tol_rot = Settings.Tolerance.tol_trans
        elif i == 3:
            control = Settings.Controller
            traj = Settings.Trajectory
            tol_trans = 0.05; tol_rot = 0.2

        target.traj = TrajectoryPlanning(robot, target.q0, target.T, t, traj)

        for j, [q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref] in enumerate(zip(target.traj.q, target.traj.q_dot, target.traj.q_dot_dot, target.traj.x, target.traj.x_dot, target.traj.x_dot_dot)):
            q = robot.Coppelia.getJointsPosition()

            if next(i for i, x in enumerate(target.traj.q) if np.array_equal(x, q_ref)) >= 103:
                pass
        
            if j >= 190:
                pass

            if isClose(robot, target.T, q, tol_trans=tol_trans, tol_rot=tol_rot):
                print(f'End-Effector is close to target {i+1}')
                break

            if q_ref is None:
                q_ref = robot.ikine_LMS(SE3.Trans(x_ref[:3])*SE3.Eul(x_ref[3:])).q 
                q_dot_ref = getDot(q_ref, q_ref0)
                q_ref0 = q_ref

            if Settings.Controller.type == 'cart':
                q_control, q_dot_control = cartesianSpaceController(robot, x_ref, x_dot_ref, q, q_dot_ref)
            elif Settings.Controller.type == 'joint':
                q_control, q_dot_control = jointSpaceController(robot, q_ref, x_dot_ref, q, q_dot_ref)
            elif Settings.Controller.type is None:
                q_dot_control = q_dot_ref
                # q_dot_control = inverseDifferentialKinematics(robot, q, x_dot_ref)
                q_control = q + q_dot_control*Settings.Ts
            
            robot.Coppelia.setJointsTargetVelocity(q_dot_control)
            robot.Coppelia.step(x_ref[:3])
            target.measures.append([
                Real(q_control, q_dot_control, None, poseToCart(robot.fkine(q_control)), None, None),
                Ref(q_ref, q_dot_ref, q_dot_dot_ref, x_ref, x_dot_ref, x_dot_dot_ref)
            ])

        target.saveData(robot)
        robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
        robot.Coppelia.step()

        if hasattr(robot.Coppelia, 'Gripper') and target.close_gripper is not None:
            robot.Coppelia.Gripper.setActuationType(target.close_gripper, shape_path = target.shape_path)
            start_time = robot.Coppelia.sim.getSimulationTime()
            while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
                robot.Coppelia.step()
                robot.Coppelia.Gripper.actuation()

robot.Coppelia.stop()

if Settings.plot:
    plotOutputs(robot.number_joints, folder = Settings.execution_path)