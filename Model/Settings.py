import numpy as np

toolbox = False
coppelia = True
gripper = True

Ts = 0.05
T_tot = 5
Kp_trans = 3
Kp_rot = 3
trans_tol = 0.005
rot_tol = 1
controller = 'cart'

class Settings():
    def __init__(self) -> None:
        self.numberJoints = self.n
        self.q0 = np.array(
            [
                -0.01647629216313362, 
                0.037338417023420334, 
                0.0009847808396443725, 
                0.07846628129482269, 
                -0.0013139393413439393, 
                0.04261644929647446, 
                0.017349982634186745
            ]
        )
        self.q = self.q0
        
        self.Ts = Ts
        self.T_tot = T_tot
        self.t = np.arange(0, self.T_tot + self.Ts, self.Ts)
        
        self.controller = controller
        self.Kp_cart = np.concatenate(
            [
                np.eye(6)[:3]*Kp_trans, 
                np.eye(6)[3:]*Kp_rot
            ]
        ) 
        # self.Kp_cart = np.eye(6)*17
        # self.Kp_cart = np.eye(6)*25
        self.Kp_joint = np.eye(7)*35
        
        self.trans_tol = trans_tol
        self.rot_tol = rot_tol

        self.toolbox = toolbox
        self.coppelia = coppelia