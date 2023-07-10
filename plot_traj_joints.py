''' Plot Expected Path from Joints Trajectory and Real Path
    Plot Expected Joints Position and Real Joints Position
    Grasp red object, initial config, place object on red bin 
'''

from Data.targets import targets as targets
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox import xplot
import time
from tkinter import messagebox # Just for messagebox that pause sim
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa(T_tot=5) # Start Toolbox model and set T_tot for each traj to target
robot.q = robot.q0 # Set joints position for toolbox environment

plot = {
    "reference_path": True,
    "real_path": False,
    "reference_joints": False,
    "real_joints": False
}

plot["path"] = True if plot["reference_path"] or plot["real_path"] else False

if plot["path"]:
    robot.env.new() # Start toolbox environment plot

ik = 'ikine_LMS' # Set inverse kinematic solver
for i, target in enumerate(targets): # Loop each target
   
    # Initial config
    q0 = robot.q
    T0 = robot.fkine(robot.q)
    
    # Final config
    T1 = SE3.Trans(
            target.x, target.y, target.z
        )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
    if ik == 'ikine_LM':
        q1 = robot.ikine_LM(T1, q0=q0, joint_limits=True, mask=[30,1,1,1,1,1]).q # TODO: Search more about mask
    elif ik == 'ikine_LMS':
        q1 = robot.ikine_LMS(T1).q

    jtraj = rtb.jtraj(q0, q1, robot.t)
    traj = [robot.fkine(x) for x in jtraj.q] # Set traj as forward kinematics matrix
    
    if plot["path"]:
        robot.env.point3D(
            [target.x, target.y, target.z],
            label=f'Target {i+1}: {target.gripperActuation} {target.object}',
            color=target.object
        ) # Plot target cartesian position
        
        rtb_jtraj_path = robot.env.path(
            traj,
            label=f'rtb.jtraj'
        ) # Plot expected trajectory path
    
    x0 = np.block([T0.t, T0.eul()]) # Set first cartesian position

    for j in range(len(traj)): # Loop expected trajectory
        
        start_time = time.time() # Debug processing time/ Uncomment line 72
        
        q = robot.q # Get joints position
        
        T_ref = traj[j] # Set reference homogenous matrix
        x_ref = np.block([T_ref.t, T_ref.eul()]) # Set reference cartesian position
        x_dot_ref = (x_ref - x0)/robot.Ts # Set reference cartesian velocity
        x0 = x_ref # Set new "first cartesian position"
        
        q_new = robot.control(x_ref, x_dot_ref, q) # Proportional Control

        if plot["path"]:
            robot.env.step(robot.Ts) # Step toolbox simulation
        
        robot.q_time.append(jtraj.q[j]) # Append to expected joints position list
        robot.q_control_time.append(q_new) # Append to real joints position list

        if robot.isClose(q1, q_new): # Check if its close enough
            break
        # print("--- %s seconds ---" % (time.time() - start_time))

    if plot["path"]:
        real_traj = [robot.fkine(x) for x in robot.q_control_time] # Calculate real trajectory path
        real_path = robot.env.path(
            real_traj,
            label=f'real jtraj'
        ) # Plot real trajectory path
    
    if plot["reference_joints"]:
        xplot(np.array(robot.q_time)) # Plot expected joints position
    if plot["real_joints"]:
        xplot(np.array(robot.q_control_time)) # Plot real joints position
    
    robot.q_time = list() # Clean expected joints position
    robot.q_control_time = list() # Clean real joints position
    
    print(f'Target {i+1}')
    # Message Box to pause simulation
    messagebox.showinfo("showinfo", "Close Joints Position (Angle/Time) windows first, then click ok")
    
    # Remove lines
    if plot["path"]:
        robot.env.clear([rtb_jtraj_path, real_path])