import math
import numpy as np
from settings import Settings
from Kinematics.measures import poseToCart

def isClose(robot, T_target, q_real, tol_trans = Settings.Tolerance.tol_trans, tol_rot = Settings.Tolerance.tol_rot): 
    target = poseToCart(T_target)
    T_real = robot.fkine(q_real)
    real = poseToCart(T_real)

    if len(target) != len(real):
        return False

    for i, (target, real) in enumerate(zip(target, real)):
        if i < 3:
            tol = tol_trans
        else:
            tol = tol_rot
        if not math.isclose(target, real, abs_tol=tol):
            return False
        
    return True

def inverseDifferentialKinematics(robot, q, x_dot, q_dot_ref):
    """ Get inverse differential kinematics"""

    J = robot.jacob0_analytical(q, 'rpy/zyx')
    J_pinv = np.linalg.pinv(J)
    q_dot = J_pinv @ x_dot
    q_ = q + q_dot*Settings.Ts
    less = np.less(q_, robot.qlim[0])
    greater = np.greater(q_, robot.qlim[1])
    if np.any(less) or np.any(greater):
        # for i in range(robot.number_joints):
        #     if less[i]:
        #         q_dot[i] = (robot.qlim[0][i]+0.1 - q[i])/Settings.Ts
        #     if greater[i]:
        #         q_dot[i] = (robot.qlim[1][i]-0.1 - q[i])/Settings.Ts
        q_dot = q_dot_ref

    return q_dot

def cartesianSpaceController(robot, x_ref, x_dot_ref, q, q_dot_ref):
    T = robot.fkine(q)
    x = poseToCart(T)

    x_err = x_ref - x
    control_signal = Settings.Controller.Kp@x_err
    control_signal = control_signal + x_dot_ref
    q_dot_control = inverseDifferentialKinematics(robot, q, control_signal, q_dot_ref)
    q_control = q + q_dot_control*Settings.Ts
    return q_control, q_dot_control

def jointSpaceController(robot, q_ref, x_dot_ref, q, q_dot_ref):
    q_dot_dot_ref = inverseDifferentialKinematics(robot, q_ref, x_dot_ref, q_dot_ref)

    q_err = q_ref - q
    q_dot_control = control_signal = Settings.Controller.Kp@q_err + q_dot_dot_ref
    q_control = q + q_dot_control*Settings.Ts
    return q_control, q_dot_control
