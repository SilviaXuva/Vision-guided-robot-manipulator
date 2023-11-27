""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController
from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = True, vision = True)

robot.Coppelia.start()

while True:
    robot.Coppelia.step()
    pick_and_place = robot.Coppelia.Vision.getTargets()

    if len(pick_and_place) > 0:
        for pp_targets in pick_and_place:
            pp_targets.append(robot.Tz)
            for target in pp_targets:
                target.pathPlanning(robot, target.T, Settings.Trajectory)
                
                x0 = np.block([target.T0.t, target.T0.eul()])  # Initial end-effector position
                
                for i in range(len(target.ref)):
                    if hasattr(robot.Coppelia, 'Drawing'):
                        robot.Coppelia.Drawing.show([target.ref[i].t[0], target.ref[i].t[1], target.ref[i].t[2]])

                    q = robot.Coppelia.getJointsPosition()
                    target.q.append(q)
                    if isClose(robot, target.T, q):
                        break
                
                    T_ref = target.ref[i]
                    x_ref = np.block([T_ref.t, T_ref.eul()])
                    x_dot_ref = (x_ref - x0)/Settings.Ts
                    x0 = x_ref
                    
                    if Settings.Controller.type == 'cart':
                        q_control_new, q_control_dot = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
                    elif Settings.Controller.type == 'joint':
                        # q0, q_control_dot = jointSpaceController(robot, q0, x_dot_ref, q)
                        pass
                    elif Settings.Controller.type is None:
                        q_control_dot = inverseDifferentialKinematics(robot, q, x_dot_ref)

                    robot.Coppelia.setJointsTargetVelocity(q_control_dot)
                    robot.Coppelia.step()

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