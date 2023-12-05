import math
import numpy as np
from settings import Settings
from Kinematics.measures import poseToCart

def isClose(robot, T_target, q_real): 
    target = poseToCart(T_target)
    T_real = robot.fkine(q_real)
    real = poseToCart(T_real)

    if len(target) != len(real):
        return False

    for i, (target, real) in enumerate(zip(target, real)):
        if i < 3:
            tol = Settings.Tolerance.tol_trans
        else:
            tol = Settings.Tolerance.tol_rot
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
    T = robot.fkine(q)
    x = poseToCart(T)

    x_err = x_ref - x
    control_signal = Settings.Controller.Kp@x_err
    control_signal = control_signal + x_dot_ref
    q_dot_control = inverseDifferentialKinematics(robot, q, control_signal)
    q_control = q + q_dot_control*Settings.Ts
    return q_control, q_dot_control

def jointSpaceController(robot, q_ref, x_dot_ref, q):
    q_dot_dot_ref = inverseDifferentialKinematics(robot, q_ref, x_dot_ref)

    q_err = q_ref - q
    q_dot_control = control_signal = Settings.Controller.Kp@q_err + q_dot_dot_ref
    q_control = q + q_dot_control*Settings.Ts
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