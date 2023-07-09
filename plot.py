''' Joints Trajectory: Grasp red object, initial config, place object on red bin '''

from Data.targets import targets1 as targets
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
import time
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa(T_tot=5)
robot.q = robot.q0

robot.env.new()

ik = 'ikine_LMS'
for i, target in enumerate(targets):
    # Initial config
    q0 = robot.q
    T0 = robot.fkine(robot.q)
    
    # Final config
    T1 = SE3.Trans(
            target.x, target.y, target.z
        )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
    if ik == 'ikine_LMS':
        q1 = robot.ikine_LMS(T1).q
    elif ik == 'ikine_min':
        q1 = robot.ikine_min(T1, q0=q0, qlim=True, stiffness=1e-3, method='Powell').q
    
    jtraj = rtb.jtraj(q0, q1, robot.t)
    traj = [robot.fkine(x) for x in jtraj.q]
    
    x0 = np.block([T0.t, T0.eul()])

    for i in range(len(traj)):
        
        start_time = time.time()
        
        q = robot.q
        
        T_ref = traj[i]
        x_ref = np.block([T_ref.t, T_ref.eul()])
        x_dot_ref = (x_ref - x0)/robot.Ts
        x0 = x_ref
        
        q_new = robot.control(x_ref, x_dot_ref, q)

        robot.env.step(robot.Ts)

        if robot.isClose(q1, q_new):
            break
        # print("--- %s seconds ---" % (time.time() - start_time))