from settings import Settings

import numpy as np
from spatialmath import SE3

class Measures():
    def __init__(self, T: np.ndarray, q=[], q_dot=[], q_dot_dot=[], x=[], x_dot=[], x_dot_dot=[]) -> None:
        self.T = T
        self.q = q
        self.q_dot = q_dot
        self.q_dot_dot = q_dot_dot
        self.x = x
        self.x_dot = x_dot
        self.x_dot_dot = x_dot_dot

class Ref(Measures):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

class Real(Measures):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

def getDot(x, x0: SE3|np.ndarray):
    if isinstance(x, list):
        dot = [
            (x[i] - x0)/Settings.Ts if i == 0 
            else (x[i] - x[i-1])/Settings.Ts 
            for i in range(len(x))
        ]
    elif isinstance(x, SE3) or isinstance(x, np.ndarray):
        dot = (x[0] - x0)/Settings.Ts

    return dot

def poseToCart(T: SE3):
    x = np.block([T.t, T.rpy()])
    return x

def cartToPose(x: np.ndarray):
    T = SE3.Trans(x[:,3])*SE3.RPY(x[3:])
    return T