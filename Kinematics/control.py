from settings import Settings
from Kinematics.measures import poseToCart

import numpy as np
from spatialmath import SE3, base
import math
from roboticstoolbox import DHRobot, ERobot

class Controller:
    def __init__(self, Kp = 1, type: str = 'cart') -> None:
        self.type = type
        self.Kp = Kp

    def log(self):
        return self.__dict__
    
def getTError(x: SE3, y: SE3):
    err_ = np.linalg.inv(x) @ y.A
    err = np.empty(6)

    # Translational error
    err[:3] = err_[:3, -1]

    # Angular error
    err[3:] = base.tr2rpy(err_, unit="rad", order="zyx", check=False)

    return err

def isClose(robot: DHRobot|ERobot, T_target: SE3, q_real: np.ndarray, tol: np.ndarray): 
    tol = np.array(tol)
    T = robot.fkine(q_real)
    err = getTError(T, T_target)
    
    if tol.shape == (1,1):
        return True if np.sum(np.abs(err)) < tol[0] else False
    else:
        tol_trans = tol[0]; tol_rot = tol[1]
        for i, e in enumerate(err):
            if i < 3:
                tol = tol_trans
            else:
                tol = tol_rot
            if not np.abs(e) <= tol:
                return False
        
        return True

def inverseDifferentialKinematics(robot: DHRobot|ERobot, q: np.ndarray, control_signal: np.ndarray):
    """ Get inverse differential kinematics"""

    J = robot.jacobe(q)
    J_pinv = np.linalg.pinv(J)
    q_dot = J_pinv @ control_signal
    q_ = q + q_dot*Settings.Ts
    less = np.less(q_, robot.qlim[0])
    greater = np.greater(q_, robot.qlim[1])
    if np.any(less) or np.any(greater):
        for i in range(robot.number_joints):
            if less[i]:
                q_dot[i] = (robot.qlim[0][i]+0.01 - q[i])/Settings.Ts
            if greater[i]:
                q_dot[i] = (robot.qlim[1][i]-0.01 - q[i])/Settings.Ts
    return q_dot

def cartesianSpaceController(robot: DHRobot|ERobot, Kp: np.ndarray, q: np.ndarray, T_ref: SE3, x_dot_ref: np.ndarray = None):
    T = robot.fkine(q)
    x_err = getTError(T, T_ref)
    if base.isscalar(Kp):
        Kp = Kp * np.eye(6)
    else:
        Kp = np.diag(Kp)
    control_signal = Kp@x_err
    if x_dot_ref is not None:
        control_signal = control_signal + x_dot_ref
    q_dot_control = inverseDifferentialKinematics(robot, q, control_signal)
    q_control = q + q_dot_control*Settings.Ts
    return q_control, q_dot_control

def jointSpaceController(robot: DHRobot|ERobot, Kp: np.ndarray, q_ref: np.ndarray, x_dot_ref: np.ndarray, q: np.ndarray):
    q_dot_dot_ref = inverseDifferentialKinematics(robot, q_ref, x_dot_ref)
    q_err = q_ref - q
    if base.isscalar(Kp):
        Kp = Kp * np.eye(robot.number_joints)
    else:
        Kp = np.diag(Kp)
    q_dot_control = control_signal = Kp@q_err + q_dot_dot_ref
    q_control = q + q_dot_control*Settings.Ts
    return q_control, q_dot_control
