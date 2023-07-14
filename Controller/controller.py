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
        
        x0 = np.block([T0.t, T0.eul()])  # Initial end-effector position
        
        for i in range(len(traj)):
            T_ref = traj[i]
            self.q_ref.append(ik(T_ref, **kwargs).q)

            q = self.getJointPosition()
            
            if self.controller == 'cart':
                x_ref = np.block([T_ref.t, T_ref.eul()])
                x_dot_ref = (x_ref - x0)/self.Ts
                x0 = x_ref
                q_new, q_control_dot = self.cartesianSpaceController(x_ref, x_dot_ref, q)
            elif self.controller == 'joint':
                q0, q_control_dot = self.jointSpaceController(q0, x_dot_ref, q)
            
            self.q_control.append(q_new)
            
            self.setJointTargetVelocity(q_control_dot)
            self.env.step()

            if self.isClose(T1, q_new, self.trans_tol, self.rot_tol):
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

    def isClose(self, T_target, q_real, trans_tol = 1, rot_tol = 1):
        x_tol = y_tol = z_tol = trans_tol
        rx_tol = ry_tol = rz_tol = rot_tol
        
        target = np.block([T_target.t, T_target.eul()])
        T_real = self.fkine(q_real)
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
        
        return True