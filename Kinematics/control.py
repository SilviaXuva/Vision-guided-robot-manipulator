import math
import numpy as np
from settings import Settings

control_type = Settings.Controller.type
Ts = Settings.Ts

if control_type == 'cart':
    Kp_cart = Settings.Controller.Kp_cart
elif control_type == 'joint':
    Kp_joint = Settings.Controller.Kp_joint

if Settings.Tolerance.type == 'cart':
    tol_trans = Settings.Tolerance.tol_trans
    rot_trans = Settings.Tolerance.tol_rot

def isClose(robot, T_target, q_real): 
    target = np.block([T_target.t, T_target.eul()])
    T_real = robot.fkine(q_real)
    real = np.block([T_real.t, T_real.eul()])

    if len(target) != len(real):
        return False

    for i, (target, real) in enumerate(zip(target, real)):
        if i < 3:
            tol = tol_trans
        else:
            tol = rot_trans
        if not math.isclose(target, real, abs_tol=tol):
            return False
        
    return True

def inverseDifferentialKinematics(robot, q, x_dot):
    """ Get inverse differential kinematics"""

    J = robot.jacob0_analytical(q, 'eul')
    J_pinv = np.linalg.pinv(J)
    q_dot = J_pinv @ x_dot

    return q_dot

def cartesianSpaceController(robot, x_ref, x_dot_ref, q):
    pose = robot.fkine(q)
    x = np.block([pose.t, pose.eul()])

    x_err = x_ref - x
    control_signal = Kp_cart@x_err
    control_signal = control_signal + x_dot_ref
    q_dot_control = inverseDifferentialKinematics(robot, q, control_signal)
    q_control = q + q_dot_control*Ts
    return q_control, q_dot_control

def jointSpaceController(robot, q_ref, x_dot_ref, q):
    q_dot_dot_ref = inverseDifferentialKinematics(robot, q_ref, x_dot_ref)

    q_err = q_ref - q
    q_dot_control = control_signal = Kp_joint@q_err + q_dot_dot_ref
    q_control = q + q_dot_control*Ts
    return q_control, q_dot_control

# def InverseDifferentialKinematicsAug(q, x_dot, obs, k0):
#     # k0 = 0 # Redundancy gain - 0 for standard (non collision-free) motion planning
#     J = self.jacob0_analytical(q, 'eul')
#     J_pinv = np.linalg.pinv(J.astype(np.float64))
#     dists, grads = self.DistancesAndGrads(q, obs)
#     _, index = min(dists)
#     q0_dot = k0*np.array(grads)[index,:].T
#     q_dot = J_pinv * x_dot + (np.eye(7)-J_pinv*J)*q0_dot
#     return q_dot

# def DistancesAndGrads(q, obs):
#     dists = np.array()
#     grads = np.array()
#     return dists, grads

def control(robot, target):

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
        x_dot_ref = (x_ref - x0)/Ts
        x0 = x_ref
        
        if control_type == 'cart':
            q_control, q_dot_control = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
        elif control_type == 'joint':
            # q0, q_dot_control = jointSpaceController(robot, q0, x_dot_ref, q)
            pass
        elif control_type is None:
            q_dot_control = inverseDifferentialKinematics(robot, q, x_dot_ref)

        robot.Coppelia.setJointsTargetVelocity(q_dot_control)
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