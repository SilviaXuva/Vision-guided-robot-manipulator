def Example1():
    """ Working with targets from Data.targets and Joint Trajectory"""
    
    from Data.targets import target2 as target
    import numpy as np
    from spatialmath import SE3
    import roboticstoolbox as rtb
    import time
    from ZMQ.Coppelia import startSimulation, stopSimulation
    from ZMQ.LBR_iiwa_ZMQ import Coppelia_LBR_iiwa

    robot = Coppelia_LBR_iiwa()
    
    robot.env.new()
    startSimulation()

    for idx_target in range(0, len(target)):
        # Initial config
        q0 = robot.q = robot.getJointPosition()
        T0 = robot.fkine(robot.q)
        # Final config
        T1 = SE3.Trans(target[idx_target][:3])*SE3.Rx(target[idx_target][3])*SE3.Ry(target[idx_target][4])*SE3.Rz(target[idx_target][5])
        q1 = robot.ikine_LMS(T1).q
        
        jtraj = rtb.jtraj(q0, q1, robot.t)
        traj = [robot.fkine(x) for x in jtraj.q]
        
        x0 = np.block([T0.t, T0.eul()])

        for i in range(len(traj)):
            
            start_time = time.time()
            
            q = robot.getJointPosition()
            
            T_ref = traj[i]
            x_ref = np.block([T_ref.t, T_ref.eul()])
            x_dot_ref = (x_ref - x0)/robot.Ts
            x0 = x_ref
            
            q_new = robot.control(x_ref, x_dot_ref, q)

            robot.env.step(robot.Ts)
            
            if robot.isClose(q_new, T1):
                break
            # print("--- %s seconds ---" % (time.time() - start_time))
        
        for j in range(20):
            robot.setJointTargetVelocity([0,0,0,0,0,0,0])
            
            gripperInfo = target[idx_target][-1]
            if ' - ' in gripperInfo:
                gripperObj = gripperInfo.split(' - ')[1]
            else:
                gripperObj = ''
            gripperAct = gripperInfo.split(' - ')[0]

            if hasattr(robot, 'Gripper'):
                if gripperAct == 'Close':
                    robot.Gripper.close(f'./{gripperObj}')
                elif gripperAct == 'Open':
                    robot.Gripper.open()
                
    # Stop CoppeliaSim scene simulation
    stopSimulation()
    
Example1()