def Example1():
    """ Working with targets from Data.targets, Joint Trajectory and Coppelia """
    
    from Data.targets import targets as targets
    import numpy as np
    from spatialmath import SE3
    import roboticstoolbox as rtb
    import time
    from ZMQ.Coppelia import startSimulation, stopSimulation
    from ZMQ.LBR_iiwa_ZMQ import Coppelia_LBR_iiwa

    robot = Coppelia_LBR_iiwa(T_tot=10)
    
    robot.env.new()
    startSimulation()

    for i, target in enumerate(targets):
        # Initial config
        q0 = robot.q = robot.getJointPosition()
        T0 = robot.fkine(robot.q)
        
        T1 = SE3.Trans(
            target.x, target.y, target.z
        )*SE3.Rx(target.rx)*SE3.Ry(target.ry)*SE3.Rz(target.rz)
        q1 = robot.ikine_LMS(T1).q
        
        jtraj = rtb.jtraj(q0, q1, robot.t)
        traj = [robot.fkine(x) for x in jtraj.q]
        
        point = robot.env.point3D(
            [target.x, target.y, target.z],
            label=f'Target {i+1}: {target.gripperActuation} {target.object}',
            color=target.object
        )  # Plot target cartesian position

        rtb_jtraj_path = robot.env.path(
            traj,
            label=f'rtb.jtraj'
        )  # Plot expected trajectory path
        
        x0 = np.block([T0.t, T0.eul()])

        for j in range(len(traj)):
            
            start_time = time.time()
            
            q = robot.getJointPosition()
            
            T_ref = traj[j]
            x_ref = np.block([T_ref.t, T_ref.eul()])
            x_dot_ref = (x_ref - x0)/robot.Ts
            x0 = x_ref
            
            q_new = robot.control(x_ref, x_dot_ref, q)

            robot.env.step(robot.Ts)
            
            robot.q_time.append(jtraj.q[j])  # Append to expected joints position list
            robot.q_control_time.append(q_new)  # Append to real joints position list
            
            if robot.isClose(q1, q_new):
                break
            # print("--- %s seconds ---" % (time.time() - start_time))
            
        real_traj = [robot.fkine(x) for x in robot.q_control_time]  # Calculate real trajectory path
        real_path = robot.env.path(
            real_traj,
            label=f'real jtraj'
        )
        
        robot.q_time = list()  # Clean expected joints position
        robot.q_control_time = list()  # Clean real joints position
        
        for k in range(20):
            robot.setJointTargetVelocity([0,0,0,0,0,0,0])
            
            if hasattr(robot, 'Gripper'):
                if target.gripperActuation == 'close':
                    robot.Gripper.close(f'./{target.object.title()}')
                elif target.gripperActuation == 'open':
                    robot.Gripper.open()
        
        robot.env.clear([point, rtb_jtraj_path, real_path])
        
    stopSimulation()

Example1()