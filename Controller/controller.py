import numpy as np
from spatialmath import SE3
import math

class Controller():
    def __init__(self) -> None:
        pass
    
    def control(self, x_ref, x_dot_ref, q, gripperAct = '', gripperObj = ''):

        if self.controller == 'cart':
            q_new, q_control_dot = self.cartesianSpaceController(x_ref, x_dot_ref, q)
        # elif self.controller == 'joint':
        #     q0, q_control_dot = self.JointSpaceController(q0, x_dot_ref, q)
        
        if hasattr(self, 'setJointTargetVelocity'):
            self.setJointTargetVelocity(q_control_dot)
        self.q = q_new
                
        return q_new

    def cartesianSpaceController(self, x_ref, x_dot_ref, q):
        pose = self.fkine(q)
        x = np.block([pose.t, pose.eul()])
        
        e = x_ref - x
        control_signal = self.Kp_cart@e
        control_signal = control_signal + x_dot_ref
        q_control_dot = self.inverseDifferentialKinematics(q, control_signal)
        q_new = q + q_control_dot*self.Ts
        return q_new, q_control_dot

    def jointSpaceController(self, q_dot_ref, x_dot_ref, q):
        q_dot_dot = self.inverseDifferentialKinematics(q_dot_ref, x_dot_ref)
        q_dot_new = q_dot_ref + q_dot_dot*self.Ts          #Euler integration
        q_err = q_dot_new - q
        control_signal = self.Kp_joint@q_err
        q_control_dot = control_signal + q_dot_dot
        return q_dot_new, q_control_dot

    def inverseDifferentialKinematics(self, q, x_dot):
        """ Get inverse differential kinematics"""

        J = self.jacob0_analytical(q, 'eul')
        J_pinv = np.linalg.pinv(J)
        q_dot = J_pinv @ x_dot

        return q_dot

    # def InverseDifferentialKinematicsAug(self, q, x_dot, obs, k0):
    #     # k0 = 0 # Redundancy gain - 0 for standard (non collision-free) motion planning
    #     J = self.jacob0_analytical(q, 'eul')
    #     J_pinv = np.linalg.pinv(J.astype(np.float64))
    #     dists, grads = self.DistancesAndGrads(q, obs)
    #     _, index = min(dists)
    #     q0_dot = k0*np.array(grads)[index,:].T
    #     q_dot = J_pinv * x_dot + (np.eye(7)-J_pinv*J)*q0_dot
    #     return q_dot

    # def DistancesAndGrads(self, q, obs):
    #     dists = np.array()
    #     grads = np.array()
    #     return dists, grads

    def isClose(self, q, T: SE3):
        pose = self.fkine(q)

        target = np.block([T.t, T.eul()])
        real = np.block([pose.t, pose.eul()])
        
        if len(target) != len(real):
            return False
        i = 0
        for target, real in zip(target, real):
            if i >= 3:
                abs_tol = 1.5
            else:
                abs_tol = 0.01
            if not math.isclose(target, real, abs_tol=abs_tol):
                return False
            i = i+1
        
        return True