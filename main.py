""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController, jointSpaceController
from Kinematics.trajectoryPlanning import TrajectoryPlanning
from Data.targets import initial
from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
from spatialmath import SE3
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = True)

robot.Coppelia.start()

while True:
    robot.Coppelia.step()
    pick_place_targets = robot.Coppelia.Vision.getTargets()

    if len(pick_place_targets) > 0:
        for pick_target, place_target in pick_place_targets:
            for i, target in enumerate([pick_target, place_target]):
                robot.Coppelia.step()
                target.q0 = robot.Coppelia.getJointsPosition()
                target.T0 = robot.fkine(target.q0)

                target.traj = TrajectoryPlanning(robot, target.q0, target.T, Settings.Trajectory)

                for q_ref, x_ref, x_dot_ref in zip(target.traj.q_ref, target.traj.x_ref, target.traj.x_dot_ref):
                    robot.Coppelia.x_ref = x_ref
                    if q_ref is None:
                        q_ref = robot.ikine_LMS(SE3.Trans(x_ref[:3])*SE3.Eul(x_ref[3:])).q
                    
                    q = robot.Coppelia.getJointsPosition()
                    if isClose(robot, target.T, q):
                        print(f'End-Effector is close to target {i+1}')
                        break
                    
                    if Settings.Controller.type == 'cart':
                        q_control, q_dot_control = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
                    elif Settings.Controller.type == 'joint':
                        q_control, q_dot_control = jointSpaceController(robot, q_ref, x_dot_ref, q)
                    elif Settings.Controller.type is None:
                        q_dot_control = inverseDifferentialKinematics(robot, q, x_dot_ref)
                    robot.Coppelia.setJointsTargetVelocity(q_dot_control)
                    robot.Coppelia.step()

                    target.x_ref.append(x_ref)
                    target.x_real.append(np.block([robot.fkine(q).t, robot.fkine(q).eul()]))
                    target.q_ref.append(q_ref)
                    target.q_real.append(q)

                if Settings.plot:
                    target.plot_x(robot)
                    target.plot_q(robot)
                robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0])
                robot.Coppelia.step()

                if hasattr(robot.Coppelia, 'Gripper'):
                    try:
                        if target.close_gripper is not None:
                            robot.Coppelia.Gripper.setActuationType(target.close_gripper, shape_path = target.shape_path)
                            start_time = robot.Coppelia.sim.getSimulationTime()
                            while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
                                robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0]) 
                                robot.Coppelia.Gripper.actuation()
                                robot.Coppelia.step()
                    except Exception as e:
                        print(e)
                        pass
    else:
        break

robot.Coppelia.stop()