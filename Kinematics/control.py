from Data.transformations import PoseToCart
from settings import Settings

import numpy as np
from roboticstoolbox import DHRobot, ERobot
from spatialmath import SE3, base

def IsClose(T: SE3, T1:SE3, tol: np.ndarray): 
    err = PoseToCart(SE3(np.linalg.inv(T) @ T1.A))
    tolTrans = tol[0]; tolRot = tol[1]
    for i, e in enumerate(err):
        if i < 3:
            tol = tolTrans
        else:
            tol = tolRot
        if not np.abs(e) <= tol:
            return False, err
    return True, err

def InverseDifferentialKinematics(robot: DHRobot|ERobot, q: np.ndarray, controlSignal: np.ndarray, qDotRef: np.ndarray):
    J = robot.jacob0_analytical(q, 'rpy/zyx')
    Jpinv = np.linalg.pinv(J)
    qDot = Jpinv @ controlSignal
    q_ = q + qDot*Settings.Ts
    less = np.less(q_, robot.qlim[0])
    greater = np.greater(q_, robot.qlim[1])
    if np.any(less) or np.any(greater):
        qDot = qDotRef
    return qDot

def CartesianSpaceController(robot: DHRobot|ERobot, Kp: np.ndarray, TRef: SE3, xDoTRef: np.ndarray, q: np.ndarray, qDotRef: np.ndarray):
    x_err = PoseToCart(TRef) - PoseToCart(robot.fkine(q))
    if base.isscalar(Kp):
        Kp = Kp * np.eye(6)
    else:
        Kp = np.diag(Kp)
    controlSignal = Kp@x_err
    if xDoTRef is not None:
        controlSignal = controlSignal + xDoTRef
    qDotControl = InverseDifferentialKinematics(robot, q, controlSignal, qDotRef)
    return qDotControl

def JointSpaceController(robot: DHRobot|ERobot, Kp: np.ndarray, q: np.ndarray, qRef: np.ndarray, qDotRef: np.ndarray):
    qErr = qRef - q
    if base.isscalar(Kp):
        Kp = Kp * np.eye(robot.n)
    else:
        Kp = np.diag(Kp)
    qDotControl = controlSignal = Kp@qErr + qDotRef
    return qDotControl