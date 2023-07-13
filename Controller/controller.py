import numpy as np
import math
from roboticstoolbox import xplot

class Controller():
    def __init__(self) -> None:
        pass
    
    def control(self, T0, T1, traj):
        """Proportional controller

        Args:
            T0 (SE3): Initial pose
            T1 (SE3): Final/Target pose
            traj (list): List of pose planning (SE3)
        """
        self.q_ref = list()
        self.q_control = list()

        ik = self.ikine_LM
        kwargs = {}
        
        q1 = ik(T1, **kwargs).q  # Final config
        x0 = np.block([T0.t, T0.eul()])  # Initial end-effector position
        
        for i in range(len(traj)):
            T_ref = traj[i]
            self.q_ref.append(ik(T_ref, **kwargs).q)
            
            if hasattr(self, 'getJointPosition'):
                q = self.getJointPosition()
            else:
                q = self.q
            
            if self.controller == 'cart':
                x_ref = np.block([T_ref.t, T_ref.eul()])
                x_dot_ref = (x_ref - x0)/self.Ts
                x0 = x_ref
                
                q_new, q_control_dot = self.cartesianSpaceController(x_ref, x_dot_ref, q)
                
            elif self.controller == 'joint':
                q0, q_control_dot = self.jointSpaceController(q0, x_dot_ref, q)
            
            self.q_control.append(q_new)
            
            if hasattr(self, 'setJointTargetVelocity'):
                self.setJointTargetVelocity(q_control_dot)
            self.qd = q_control_dot
            
            self.env.step()

            if self.isClose(q1, q_new):
                break

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
        q_dot_new = q_dot_ref + q_dot_dot*self.Ts #Euler integration
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

    def isClose(self, q_target, q_real, abs_tol=0.169):
       
        if len(q_target) != len(q_real):
            return False

        for q_target, q_real in zip(q_target, q_real):
            if not math.isclose(q_target, q_real, abs_tol=abs_tol):
                return False
        
        return True