# import matplotlib.pyplot as plt
# import os
# import pandas as pd
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from settings import Settings

def TrajectoryPlanning(robot, q0, T1, trajectory):
    if trajectory.type == 'cart':
        if trajectory.source == 'rtb':
            ctraj = rtb.ctraj(robot.fkine(q0), T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
            ctraj = [SE3(pose) for pose in ctraj.A]  # Transform each pose into SE3
        elif trajectory.source == 'custom':
            ctraj = quinticEndEffectorTraj(robot.fkine(q0), T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
            ctraj = [SE3.Trans(x[:3])*SE3.Eul(x[3:]) for x in ctraj.x]  # Transform each pose into SE3
        return ctraj
    elif trajectory.type == 'joint':
        if trajectory.source == 'rtb':
            jtraj = rtb.jtraj(q0, robot.ikine_LMS(T1).q, Settings.t)  # Calculate reference joints trajectory
        elif trajectory.source == 'custom':
            jtraj = quinticJointTraj(q0, robot.ikine_LMS(T1).q, Settings.t)  # Calculate reference joints trajectory
        jtraj = [SE3(robot.fkine(q)) for q in jtraj.q]  # Transform each joint position into SE3 through forward kinematics
        return jtraj

class cart:
    def __init__(self, x, xd, xdd) -> None:
        self.x = x
        self.xd = xd
        self.xdd = xdd

class joint:
    def __init__(self, q, qd, qdd) -> None:
        self.q = q
        self.qd = qd
        self.qdd = qdd

def quinticEndEffectorTraj(T0, T1, t):
    x0 = np.block([T0.t, T0.eul()])
    x1 = np.block([T1.t, T1.eul()])
        
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
    
    x   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    xd  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    xdd = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return cart(x, xd, xdd)

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
    
    q   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    qd  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    qdd = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return joint(q, qd, qdd)
