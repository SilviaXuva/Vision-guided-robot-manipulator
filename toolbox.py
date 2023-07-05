from Data.targets import target3 as target
from spatialmath import SE3
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa()

T0 = robot.fkine(robot.q)
T1 = SE3.Trans(target[0][:3])*SE3.Rx(target[0][3])*SE3.Ry(target[0][4])*SE3.Rz(target[0][5])

jtraj = robot.jtraj(T0, T1, robot.t)
robot.plot(jtraj.q)