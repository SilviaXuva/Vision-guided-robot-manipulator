import numpy as np
from Data.targets import target3 as target
from spatialmath import SE3
import roboticstoolbox as rtb
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa()
robot.q = robot.q0

T0 = robot.fkine(robot.q)
T1 = SE3.Trans(target[0][:3])*SE3.Rx(target[0][3])*SE3.Ry(target[0][4])*SE3.Rz(target[0][5])

q0 = robot.q
q1 = robot.ikine_min(T1, q0=q0, qlim=True, stiffness=1e-3, method='Powell').q

jtraj2 = rtb.jtraj(q0, q1, robot.t)
traj = [robot.fkine(x) for x in jtraj2.q]

x0 = np.block([T0.t, T0.eul()])

robot.env.new()

for i in range(len(traj)):
    q = robot.q
    
    T_ref = traj[i]
    x_ref = np.block([T_ref.t, T_ref.eul()])
    x_dot_ref = (x_ref - x0)/robot.Ts
    x0 = x_ref
    
    q_new = robot.control(x_ref, x_dot_ref, q)

    robot.env.step()
    
    if robot.isClose(q_new, T1):
        break