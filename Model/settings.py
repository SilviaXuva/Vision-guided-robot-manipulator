import numpy as np

class Controller:
    def __init__(self, type, Kp) -> None:
        self.type = type
        if type == 'cart':
            self.Kp_trans = Kp[0]
            self.Kp_rot = Kp[1]
        elif type == 'joint':
            for i in range(len(Kp)):
                setattr(self, f'Kp_joint_{i}', Kp[i])

class Tolerance:
    def __init__(self, type, tol) -> None:
        self.type = type
        if type == 'cart':
            self.tol_trans = tol[0]
            self.tol_rot = tol[1]
        elif type == 'joint':
            for i in range(len(tol)):
                setattr(self, f'Kp_joint_{i}', tol[i])

class Settings():
    environments = [
        'coppelia',
        # 'pyplot'
    ]
    gripper = True
    vision = False

    Ts = 0.05
    T_tot = 5
    t = np.arange(0, T_tot + Ts, Ts)
    Tolerance = Tolerance('cart', [0.005, 1])
    Controller = Controller('cart', [8,6])

    q0 = np.array(
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