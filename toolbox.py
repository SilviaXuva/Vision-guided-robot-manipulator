from Data.targets import target3 as target
from spatialmath import SE3
import roboticstoolbox as rtb
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa()

T0 = robot.fkine(robot.q)
T1 = SE3.Trans(target[0][:3])*SE3.Rx(target[0][3])*SE3.Ry(target[0][4])*SE3.Rz(target[0][5])

q0 = robot.q
q1 = robot.ikine_LM(T1, q0).q
jtraj = rtb.jtraj(q0, q1, robot.t)
robot.plot(jtraj.q)