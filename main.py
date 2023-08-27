""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Kinematics.control import isClose, inverseDifferentialKinematics, cartesianSpaceController
from Coopelia.robotSimulator import RobotSimulator
from Data.targets import pick_place_target_red as targets
from Models import DH_LBR_iiwa as LBR_iiwa
import numpy as np
import roboticstoolbox as rtb
from settings import Settings

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = False, vision = True)

robot.Coppelia.start()

for i, target in enumerate(targets):
    
    target.pathPlanning(robot, target.T, Settings.Trajectory)
    
    x0 = np.block([target.T0.t, target.T0.eul()])  # Initial end-effector position
    
    for i in range(len(target.ref)):
        if hasattr(robot.Coppelia, 'Drawing'):
            robot.Coppelia.Drawing.show([target.ref[i].t[0], target.ref[i].t[1], target.ref[i].t[2]])

        if hasattr(robot.Coppelia, 'Vision'):
            robot.Coppelia.Vision.GetImg()

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

        # if i >= len(traj) - 10:
        #     print()
    
    if hasattr(robot.Coppelia, 'Gripper'):
        try:
            robot.Coppelia.Gripper.setActuationType(target.close_gripper, shape_path = target.shape_path)
            start_time = robot.Coppelia.sim.getSimulationTime()
            while robot.Coppelia.sim.getSimulationTime() - start_time < 2:
                robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0]) 
                robot.Coppelia.Gripper.actuation()
                robot.Coppelia.step()
        except Exception as e:
            print(e)
            pass
    
    target.plot(save=True)

robot.Coppelia.stop()