import numpy as np
from roboticstoolbox import DHRobot

def quinticEndEffectorTraj(T0, T1, t):
    x0 = np.block([T0.t, T0.eul()])
    xf = np.block([T1.t, T1.eul()])
        
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
            [xf[i]],
            [0],
            [0]
        ])

        C = np.append(C, np.linalg.solve(T, Q), axis=1)
        
    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    x   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    xd  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    xdd = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return x, xd, xdd

def quinticJointTraj(number_joints, T0, T1, t, ik = DHRobot.ikine_LMS, **kwargs):
    q0 = ik(T0, **kwargs).q
    qf = ik(T1, **kwargs).q
        
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
    for i in range(number_joints):
        Q = np.array([
        # ---Initial---
            [q0[i]],
            [0],
            [0],
        # ---Final---
            [qf[i]],
            [0],
            [0]
        ])

        C = np.append(C, np.linalg.solve(T, Q), axis=1)

    tt = np.array([np.ones(t.shape), t, t**2, t**3, t**4, t**5]).T
    
    q   = tt @ np.array([1*C[0], 1*C[1], 1*C[2], 1*C[3],  1*C[4],  1*C[5]])
    qd  = tt @ np.array([0*C[0], 1*C[1], 2*C[2], 3*C[3],  4*C[4],  5*C[5]])
    qdd = tt @ np.array([0*C[0], 0*C[1], 2*C[2], 6*C[3], 12*C[4], 20*C[5]])

    return q, qd, qdd
