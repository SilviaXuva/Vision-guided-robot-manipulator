from settings import Settings
from Kinematics.measures import Ref, getDot, poseToCart, cartToPose

import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, ERobot

class CartesianTrajectory(Ref):
    def __init__(self, robot, q0, T, x, x_dot = None, x_dot_dot = None) -> None:
        if x_dot is None:
            x_dot = getDot(x, poseToCart(robot.fkine(q0)))
        if x_dot_dot is None:
            x_dot_dot = getDot(x_dot, poseToCart(robot.fkine(q0)))
        q = q_dot = q_dot_dot = [None for _ in x]
        super().__init__(T, q, q_dot, q_dot_dot, x, x_dot, x_dot_dot)

class JointTrajectory(Ref):
    def __init__(self, robot, q0, T, q, q_dot, q_dot_dot) -> None:
        if T is None:
            T = [robot.fkine(q) for q in q]
        x = [poseToCart(robot.fkine(q)) for q in q]
        x_dot = getDot(x, poseToCart(robot.fkine(q0)))
        x_dot_dot = getDot(x_dot, poseToCart(robot.fkine(q0)))
        super().__init__(T, q, q_dot, q_dot_dot, x, x_dot, x_dot_dot)

class Trajectory:
    def __init__(self, type: str = 'joint', source: str = 'rtb', t: np.ndarray = np.arange(0, Settings.T_tol + Settings.Ts, Settings.Ts)) -> None:
        self.type = type
        self.source = source
        self.t = t
    
    def log(self):
        return dict({(k):({'t0':v[0], 't1':v[-1], 'Ts': v[1]-v[0], 't_len': len(v)} if k == 't' else v) for k,v in self.__dict__.items()})

def TrajectoryPlanning(robot: DHRobot|ERobot, target): 
    if target.Trajectory.type == 'cart':
        if target.Trajectory.source == 'rtb':
            traj = rtb.ctraj(robot.fkine(target.q0), target.T, target.Trajectory.t) # Calculate reference cartesian/end-effector trajectory
            x_ref = [poseToCart(SE3(T_ref)) for T_ref in traj.A]; x_dot_ref = None; x_dot_dot_ref = None
            T_ref = [SE3(arr) for arr in traj.A]
        elif target.Trajectory.source == 'custom':
            x_ref, x_dot_ref, x_dot_dot_ref = quinticEndEffectorTraj(robot.fkine(target.q0), target.T, target.Trajectory.t) # Calculate reference cartesian/end-effector trajectory
            T_ref = [cartToPose(x) for x in x_ref]
        traj = CartesianTrajectory(robot, target.q0, T_ref, x_ref, x_dot_ref, x_dot_dot_ref)
    else:
        if target.Trajectory.source == 'rtb':
            traj = rtb.jtraj(target.q0, robot.ikine_LMS(target.T).q, target.Trajectory.t)  # Calculate reference joints trajectory
            q_ref = traj.q; q_dot_ref = traj.qd; q_dot_dot_ref = traj.qdd
        elif target.Trajectory.source == 'custom':
            q_ref, q_dot_ref, q_dot_dot_ref = quinticJointTraj(target.q0, robot.ikine_LMS(target.T).q, target.Trajectory.t)  # Calculate reference joints trajectory
        traj = JointTrajectory(robot, target.q0, None, q_ref, q_dot_ref, q_dot_dot_ref)

    Settings.log('Target:', target.T.A, 'rpy', target.T.rpy())
    Settings.log('Gripper:', target.Gripper_actuation.log())
    Settings.log('Controller:', target.Controller.log())
    Settings.log('Trajectory:', target.Trajectory.log())
    Settings.log('Tolerance:', target.tolerance)     

    return traj

def quinticEndEffectorTraj(T0, T1, t):
    x0 = poseToCart(T0)
    x1 = poseToCart(T1)
        
    # Matrix T
    T = np.array([
    # ---Initial---
        [1,     min(t),     min(t)**2,      min(t)**3,      min(t)**4,          min(t)**5       ],
        [0,     1,          2*min(t),       3*min(t)**2,    4*min(t)**3,        5*min(t)**4     ],
        [0,     0,          2,              6*min(t),       12*min(t)**2,       20*min(t)**3    ],
    # ---Final---
        [1,     max(t),     max(t)**2,      max(t)**3,      max(t)**4,          max(t)**5       ],
        [0,     1,          2*max(t),       3*max(t)**2,    4*max(t)**3,        5*max(t)**4     ],
        [0,     0,          2,              6*max(t),       12*max(t)**2,       20*max(t)**3    ]
    ])
    # Matrix of coefficients
    C = np.zeros(shape=[6, 0])
    for i in range(6):
        Q = np.array([
        # ---Initial---
            [x0[i]],
            [0]
        # ---Final---
            [x1[i]],
            [0],
            [0]
        ])

        C = np.append(C, np.linalg.solve(T, Q), axis=1)
        
    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    x_ref   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    x_dot_ref  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    x_dot_dot_ref = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return x_ref, x_dot_ref, x_dot_dot_ref

def quinticJointTraj(q0, q1, t):
    # Matrix T
    T = np.array([
    # ---Initial---
        [1,     min(t),     min(t)**2,      min(t)**3,      min(t)**4,          min(t)**5       ],
        [0,     1,          2*min(t),       3*min(t)**2,    4*min(t)**3,        5*min(t)**4     ],
        [0,     0,          2,              6*min(t),       12*min(t)**2,       20*min(t)**3    ],
    # ---Final---
        [1,     max(t),     max(t)**2,      max(t)**3,      max(t)**4,          max(t)**5       ],
        [0,     1,          2*max(t),       3*max(t)**2,    4*max(t)**3,        5*max(t)**4     ],
        [0,     0,          2,              6*max(t),       12*max(t)**2,       20*max(t)**3    ]
    ])
    # Matrix of coefficients
    C = np.zeros(shape=[6, 0])
    for i in range(len(q0)):
        Q = np.array([
        # ---Initial---
            [q0[i]],
            [0],
            [0],
        # ---Final---
            [q1[i]],
            [0],
            [0]
        ])

        C = np.append(C, np.linalg.solve(T, Q), axis=1)

    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    q_ref   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    q_dot_ref  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    q_dot_dot_ref = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return q_ref, q_dot_ref, q_dot_dot_ref
