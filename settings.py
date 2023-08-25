from datetime import datetime
import numpy as np
import os

class Controller:
    def __init__(self, type, Kp = []) -> None:
        self.type = type
        if type == 'cart':
            Kp_trans = Kp[0]
            Kp_rot = Kp[1]
            self.Kp_cart = np.concatenate(
            [
                np.eye(6)[:3]*Kp_trans, 
                np.eye(6)[3:]*Kp_rot
            ]
        ) 
        elif type == 'joint':
            self.Kp_joint = np.diag(Kp)

class Tolerance:
    def __init__(self, type, tol) -> None:
        self.type = type
        if type == 'cart':
            self.tol_trans = tol[0]
            self.tol_rot = tol[1]
        elif type == 'joint':
            for i in range(len(tol)):
                setattr(self, f'tol_joint_{i}', tol[i])

class Trajectory:
    def __init__(self, type, source) -> None:
        self.type = type
        self.source = source

class Settings:
    Ts = 0.05
    T_tot = 5
    t = np.arange(0, T_tot + Ts, Ts)
    
    Trajectory = Trajectory('cart', 'rtb')
    # Controller = Controller(None)
    Controller = Controller('cart', [8,6])
    # Controller = Controller('joint', 15)
    Tolerance = Tolerance('cart', [0.005, 1])
    
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

    output_path = fr'.\Output'
    execution_path = fr'{output_path}\Trajectory type = {Trajectory.type} - source = {Trajectory.source}\Controller type = {Controller.type}'
    os.makedirs(execution_path, exist_ok=True)
    start_time = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")