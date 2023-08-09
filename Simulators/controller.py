import math
from Model.settings import Settings
import numpy as np

Kp_cart = np.concatenate(
    [
        np.eye(6)[:3]*Settings.Kp_trans, 
        np.eye(6)[3:]*Settings.Kp_rot
    ]
) 
# Kp_cart = np.eye(6)*17
# Kp_cart = np.eye(6)*25
Kp_joint = np.eye(7)*35

def control(simulator, T0, T1, traj):
    """Proportional controller

    Args:
        T0 (SE3): Initial pose
        T1 (SE3): Final/Target pose
        traj (list): List of pose (SE3)
    """
    simulator.plot_ref(T1, traj)

    x0 = np.block([T0.t, T0.eul()])  # Initial end-effector position
    
    for i in range(len(traj)):
        q = simulator.getJointPosition()
        
        if Settings.controllerType == 'cart':
            T_ref = traj[i]
            x_ref = np.block([T_ref.t, T_ref.eul()])
            x_dot_ref = (x_ref - x0)/Settings.Ts
            x0 = x_ref
            q_new, q_control_dot = cartesianSpaceController(simulator, x_ref, x_dot_ref, q)
        elif Settings.controller == 'joint':
            q0, q_control_dot = jointSpaceController(simulator, q0, x_dot_ref, q)
        
        simulator.setJointTargetVelocity(q_control_dot)
        
        if isClose(simulator, T1, q_new, Settings.trans_tol, Settings.rot_tol):
            break
        
        # if i >= len(traj) - 10:
        #     print()
    
    simulator.setJointTargetVelocity([0,0,0,0,0,0,0])
    simulator.plot_real(block = False)

def cartesianSpaceController(simulator, x_ref, x_dot_ref, q):
    pose = simulator.robot.fkine(q)
    x = np.block([pose.t, pose.eul()])
    
    e = x_ref - x
    control_signal = Kp_cart@e
    control_signal = control_signal + x_dot_ref
    q_control_dot = inverseDifferentialKinematics(simulator, q, control_signal)
    q_new = q + q_control_dot*Settings.Ts
    return q_new, q_control_dot

def jointSpaceController(q_dot_ref, x_dot_ref, q):
    q_dot_dot = inverseDifferentialKinematics(q_dot_ref, x_dot_ref)
    q_dot_new = q_dot_ref + q_dot_dot*Settings.Ts #Euler integration
    q_err = q_dot_new - q
    control_signal = Kp_joint@q_err
    q_control_dot = control_signal + q_dot_dot
    return q_dot_new, q_control_dot

def inverseDifferentialKinematics(simulator, q, x_dot):
    """ Get inverse differential kinematics"""

    J = simulator.robot.jacob0_analytical(q, 'eul')
    J_pinv = np.linalg.pinv(J)
    q_dot = J_pinv @ x_dot

    return q_dot

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

def isClose(simulator, T_target, q_real, trans_tol = 1, rot_tol = 1):
    x_tol = y_tol = z_tol = trans_tol
    rx_tol = ry_tol = rz_tol = rot_tol
    
    target = np.block([T_target.t, T_target.eul()])
    T_real = simulator.robot.fkine(q_real)
    real = np.block([T_real.t, T_real.eul()])

    if len(target) != len(real):
        return False

    for i, (target, real) in enumerate(zip(target, real)):
        if i < 3:
            tol = trans_tol
        else:
            tol = rot_tol
        if not math.isclose(target, real, abs_tol=tol):
            return False
        
    # trans_tol = 0.005
    # rot_tol = 1

    # target_ = np.block([T_target.t, T_target.eul()])
    # T_real = self.fkine(q_real)
    # real_ = np.block([T_real.t, T_real.eul()])

    # for i, (target, real) in enumerate(zip(target_, real_)):
    #     if i < 3:
    #         tol = trans_tol
    #     else:
    #         tol = rot_tol
    #     print(i, math.isclose(target, real, abs_tol=tol))
        
    return True
