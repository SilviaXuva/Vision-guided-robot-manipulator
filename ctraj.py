""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Data.targets import all_pick_place_without_initial as targets
from Model.LBR_iiwa import LBR_iiwa
import roboticstoolbox as rtb
from spatialmath import SE3

robot = LBR_iiwa()

robot.startSimulation()

for i, target in enumerate(targets):
    # Initial config
    T0 = robot.fkine(robot.getJointPosition())
    
    # Final pose
    T1 = SE3.Trans(
        target.x, target.y, target.z
    )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
    
    ctraj = rtb.ctraj(T0, T1, robot.t)  # Calculate reference cartesian trajectory
    traj = [SE3(pose) for pose in ctraj.A]  # Transform each pose into SE3

    # robot.env.plot_ref(T1, traj)  # Plot target pose and reference trajectory path
    
    robot.control(T0, T1, traj) # Control
    
    # robot.env.plot_real(target)  # Plot real trajectory path and joints

robot.stopSimulation()