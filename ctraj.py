def Example1():
    """ Working with targets from Data.targets, Cartesian Trajectory and Coppelia """
    
    from Data.targets import targets as targets
    from spatialmath import SE3
    import roboticstoolbox as rtb
    from Toolbox.LBR_iiwa_DH import LBR_iiwa

    robot = LBR_iiwa(T_tot = 5, Kt = 1, Kr = 1)
    
    robot.env.new()
    # startSimulation()

    for i, target in enumerate(targets):
        # Initial config
        T0 = robot.fkine(robot.getJointPosition())
        
        # Final pose
        T1 = SE3.Trans(
            target.x, target.y, target.z
        )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
        
        ctraj = rtb.ctraj(T0, T1, robot.t)  # Calculate reference cartesian trajectory
        traj = [SE3(pose) for pose in ctraj.A]  # Transform each pose into SE3

        robot.env.plot_ref(target, T1, traj)  # Plot target pose and reference trajectory path
        
        robot.control(T0, T1, traj) # Control
        
        robot.env.plot_real(target)  # Plot real trajectory path and joints
        
    # stopSimulation()

Example1()