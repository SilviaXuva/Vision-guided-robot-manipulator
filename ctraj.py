def Example1():
    """ Working with targets from Data.targets, Cartesian Trajectory and Coppelia """
    
    from Data.targets import targets as targets
    import numpy as np
    from spatialmath import SE3
    import roboticstoolbox as rtb
    from Toolbox.LBR_iiwa_DH import LBR_iiwa

    robot = LBR_iiwa(T_tot=5)
    
    robot.env.new()
    # startSimulation()

    for i, target in enumerate(targets):
        # Initial config
        q0 = robot.q  # robot.getJointPosition() 
        T0 = robot.fkine(q0)
        
        # Final pose
        T1 = SE3.Trans(
            target.x, target.y, target.z
        )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
        
        ctraj = rtb.ctraj(T0, T1, robot.t)
        traj = [SE3(pose) for pose in ctraj.A]

        robot.env.pose(
            T1
        )  # Plot target pose
        robot.env.path(
            traj,
            label=f'Reference path',
            save_path = fr'{robot.output_path}\{target.gripperActuation.title()} {target.object}_Reference path.png'
        )  # Plot reference trajectory path
        
        robot.control(T0, T1, traj)
        
        real_traj = [robot.fkine(q_control) for q_control in robot.q_control]  # Calculate real trajectory path
        robot.env.path(
            real_traj,
            label=f'Real path',
            save_path = fr'{robot.output_path}\{target.gripperActuation.title()} {target.object}_Real path.png'
        ) # Plot real trajectory path
        
        robot.env.joints(
            np.array(robot.q_ref),
            np.array(robot.q_control),
            save_path = fr'{robot.output_path}\{target.gripperActuation.title()} {target.object}_Joints ref+real.png',
            block = False
        ) # Plot joints
        
        self.env.clear()
        
    # stopSimulation()

Example1()