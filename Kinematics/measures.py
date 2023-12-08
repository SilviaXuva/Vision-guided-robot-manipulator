from settings import Settings
import numpy as np

class Measures():
    def __init__(self, q=[], q_dot=[], q_dot_dot=[], x=[], x_dot=[], x_dot_dot=[]) -> None:
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

def getDot(x, x0):
    if len(np.array([x]).shape) == 2 and np.array([x]).shape[0] == 1:
        dot = (x[0] - x0)/Settings.Ts
    else:
        dot = [
            (x[i] - x0)/Settings.Ts if i == 0 
            else (x[i] - x[i-1])/Settings.Ts 
            for i in range(len(x))
        ]
    return dot

def poseToCart(T):
    return np.block([T.t, T.rpy()])