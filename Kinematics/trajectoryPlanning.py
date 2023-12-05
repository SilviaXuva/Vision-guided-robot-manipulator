# import matplotlib.pyplot as plt
# import os
# import pandas as pd
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from settings import Settings
from Kinematics.measures import Ref, Real, getDot, poseToCart

class CartesianTrajectory(Ref):
    def __init__(self, x, x_dot, x_dot_dot) -> None:
        q = q_dot = q_dot_dot = [None for _ in x]
        super().__init__(q, q_dot, q_dot_dot, x, x_dot, x_dot_dot)

class JointTrajectory(Ref):
    def __init__(self, robot, q0, q, q_dot, q_dot_dot) -> None:
        x = [poseToCart(robot.fkine(q)) for q in q]
        x_dot = getDot(x, poseToCart(robot.fkine(q0)))
        x_dot_dot = getDot(x_dot, poseToCart(robot.fkine(q0)))
        super().__init__(q, q_dot, q_dot_dot, x, x_dot, x_dot_dot)

def TrajectoryPlanning(robot, q0, T1, trajectory):
    if trajectory.type == 'cart':
        if trajectory.source == 'rtb':
            traj = rtb.ctraj(robot.fkine(q0), T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
            x_ref = [poseToCart(SE3(T_ref)) for T_ref in traj.A]
            x_dot_ref = getDot(x_ref, poseToCart(robot.fkine(q0)))
            x_dot_dot_ref = getDot(x_dot_ref, poseToCart(robot.fkine(q0)))
        elif trajectory.source == 'custom':
            x_ref, x_dot_ref, x_dot_dot_ref = quinticEndEffectorTraj(robot.fkine(q0), T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
        traj = CartesianTrajectory(x_ref, x_dot_ref, x_dot_dot_ref)
    else:
        if trajectory.source == 'rtb':
            traj = rtb.jtraj(q0, robot.ikine_LMS(T1).q, Settings.t)  # Calculate reference joints trajectory
            q_ref = traj.q; q_dot_ref = traj.qd; q_dot_dot_ref = traj.qdd
        elif trajectory.source == 'custom':
            q_ref, q_dot_ref, q_dot_dot_ref = quinticJointTraj(q0, robot.ikine_LMS(T1).q, Settings.t)  # Calculate reference joints trajectory
        traj = JointTrajectory(robot, q0, q_ref, q_dot_ref, q_dot_dot_ref)

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
            [0],
            [0],
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
