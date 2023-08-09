import numpy as np

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
    Kp_trans = 8
    Kp_rot = 6
    trans_tol = 0.005
    rot_tol = 1
    controllerType = 'cart'

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

