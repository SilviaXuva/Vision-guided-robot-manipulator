from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart, GetDot
from Models import DH_LBR_iiwa
from Helpers.measures import Real, Ref
from Kinematics.control import CartesianSpaceController, JointSpaceController, InverseDifferentialKinematics, IsClose
from Kinematics.trajectory import TrajectoryPlanning
from settings import Settings
from Simulators import CoppeliaSim

import numpy as np
import roboticstoolbox as rtb

robotDh = DH_LBR_iiwa()
coppelia = CoppeliaSim(robotDh, drawing=True, gripper=True, camera=True, arucoVision = True, createCuboids=True)

coppelia.Start()

while len(coppelia.ArucoVision.detected) > 0 or coppelia.Cuboids.created < coppelia.Cuboids.maxCreation:
    if not coppelia.moveRobot:
        coppelia.Step()
    else:
        for marker in coppelia.ArucoVision.detected:
            coppelia.ArucoVision.PrintEstimatePose(marker)
            pickPlace = GetArucoPickPlace(robotDh, marker)
            if pickPlace is None:
                coppelia.Step()
            else:
                success = True
                align, pick, place, initial = pickPlace
                for target in [align, pick, place, initial]:
                    if not success:
                        success = True
                    else:
                        qRef0 = q0 = coppelia.GetJointsPosition()
                        traj = TrajectoryPlanning(
                            type = target.trajectoryType, 
                            source = target.trajectorySource, 
                            robot = robotDh, 
                            q0 = q0, 
                            T1 = target.T, 
                            t = target.t
                        )
                        for TRef, qRef, qDotRef, xRef, xDotRef in zip(traj.T, traj.q, traj.qDot, traj.x, traj.xDot):
                            q = coppelia.GetJointsPosition()
                            if qRef is None:
                                qRef = robotDh.ikine_LMS(TRef).q
                                qDotRef = GetDot([qRef], qRef0)[0]
                                qRef0 = qRef
                            if target.controller == 'cart':
                                _, qDotControl = CartesianSpaceController(robotDh, target.Kp, q, TRef, xDotRef, qDotRef)
                            elif target.controller == 'joint':
                                _, qDotControl = JointSpaceController(robotDh, target.Kp, q, qRef, qDotRef)
                            elif target.controller is None:
                                qDotControl = InverseDifferentialKinematics()
                            elif target.controller == 'rtb':
                                v, _ = rtb.p_servo(robotDh.fkine(q), TRef, target.Kp)
                                qDotControl = np.linalg.pinv(robotDh.jacobe(q)) @ v
                            qControl = q + qDotControl*Settings.Ts
                            target.measures.append([
                                Real(qControl, qDotControl, None, PoseToCart(robotDh.fkine(qControl)), None, None),
                                Ref(qRef, None, None, xRef, None, None)
                            ])
                            target.SaveData(robotDh)
                            coppelia.SetJointsTargetVelocity(qDotControl); coppelia.Step(xRef[:3])
                            close, e = IsClose(robotDh.fkine(qControl), target.T, tol = target.tol)
                            if close:
                                Settings.Log(f'End-effector is close to target {target.name}')
                                break
                        coppelia.SetJointsTargetVelocity([0,0,0,0,0,0,0]); coppelia.Step(xRef[:3])
                        if hasattr(coppelia, 'Gripper') and target.GripperActuation.close is not None:
                            success = coppelia.Gripper.SetupActuation(target.GripperActuation)
                            if success:
                                for i in range(40):
                                    coppelia.Step()
                                    coppelia.Gripper.Actuation()
                        else:
                            success = True
        coppelia.moveRobot = False

coppelia.Stop()
