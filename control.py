import math
import numpy as np
from settings import Settings

control_type = Settings.Controller.type
Ts = Settings.Ts

Kp_cart = np.concatenate(
    [
        np.eye(6)[:3]*Settings.Controller.Kp_trans, 
        np.eye(6)[3:]*Settings.Controller.Kp_rot
    ]
) 
Kp_joint = np.eye(7)*35

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
    
    e = x_ref - x
    control_signal = Kp_cart@e
    control_signal = control_signal + x_dot_ref
    q_control_dot = inverseDifferentialKinematics(robot, q, control_signal)
    q_new = q + q_control_dot*Ts
    return q_new, q_control_dot

def jointSpaceController(q_dot_ref, x_dot_ref, q):
    q_dot_dot = inverseDifferentialKinematics(q_dot_ref, x_dot_ref)
    q_dot_new = q_dot_ref + q_dot_dot*Ts #Euler integration
    q_err = q_dot_new - q
    control_signal = Kp_joint@q_err
    q_control_dot = control_signal + q_dot_dot
    return q_dot_new, q_control_dot

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

        q = robot.Coppelia.getJointsPosition()
        
        if control_type == 'cart':
            T_ref = target.ref[i]
            x_ref = np.block([T_ref.t, T_ref.eul()])
            x_dot_ref = (x_ref - x0)/Ts
            x0 = x_ref
            q_new, q_control_dot = cartesianSpaceController(robot, x_ref, x_dot_ref, q)
        elif control_type == 'joint':
            q0, q_control_dot = jointSpaceController(robot, q0, x_dot_ref, q)
        
        target.q.append(q_new)
        
        robot.Coppelia.setJointsTargetVelocity(q_control_dot)
        robot.Coppelia.step()
        
        if isClose(robot, target.T, q_new):
            break
        
        # if i >= len(traj) - 10:
        #     print()
    
    if hasattr(robot.Coppelia, 'Gripper'):
        robot.Coppelia.Gripper.setActuationType(target.closeGripper, shapePath = target.shapePath)
        startTime = robot.Coppelia.sim.getSimulationTime()
        while robot.Coppelia.sim.getSimulationTime() - startTime < 2:
            robot.Coppelia.setJointsTargetVelocity([0,0,0,0,0,0,0]) 
            robot.Coppelia.Gripper.actuation()
            robot.Coppelia.step()
