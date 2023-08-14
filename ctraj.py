""" Working with targets from Data.targets and Toolbox Cartesian Trajectory """

from Data.targets import all_pick_place_without_initial as targets
from Model.LBR_iiwa import LBR_iiwa
from Model.settings import Settings
import roboticstoolbox as rtb
from Simulators.control import control
from Simulators.simulation import Simulator
from spatialmath import SE3

robot = LBR_iiwa()
simulator = Simulator(robot)

simulator.start()

for i, target in enumerate(targets):
    # Initial config
    T0 = robot.fkine(simulator.getJointPosition())
    
    # Final pose
    T1 = SE3.Trans(
        target.x, target.y, target.z
    )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
    
    ctraj = rtb.ctraj(T0, T1, Settings.t)  # Calculate reference cartesian trajectory
    traj = [SE3(pose) for pose in ctraj.A]  # Transform each pose into SE3
    
    control(simulator, T0, T1, traj) # Control
    
    simulator.Gripper.actuation(target.gripperClose, shapePath = target.shapePath)

simulator.stop()