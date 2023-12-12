from settings import Settings
from Models import DH_LBR_iiwa
from Simulators import CoppeliaSim
from VisionProcessing.aruco import ArucoVision
from Kinematics.trajectory import TrajectoryPlanning
from Kinematics.control import CartesianSpaceController, JointSpaceController, InverseDifferentialKinematics, IsClose
from Data.targets import GetArucoPickPlace
from Data.transformations import PoseToCart
from Helpers.measures import Real, Ref

import roboticstoolbox as rtb
import numpy as np

robotDh = DH_LBR_iiwa()
coppelia = CoppeliaSim(robotDh, drawing=True, gripper=True, camera=True, createCuboids=False)
vision = ArucoVision(coppelia.Camera)

coppelia.Start()

while True:
    vision.Process()
    if coppelia.moveRobot:
        if len(vision.detected) > 0:
            for marker in vision.detected:
                vision.PrintEstimatePose(marker)
                alignPickPlace = GetArucoPickPlace(marker)
                if alignPickPlace is not None:
                    align, pick, place = alignPickPlace
                    for target in pick, place:
                        q0 = coppelia.GetJointsPosition()
                        traj = TrajectoryPlanning(
                            type = target.trajectoryType, 
                            source = target.trajectorySource, 
                            robot = robotDh, 
                            q0 = q0, 
                            T1 = target.T, 
                            t = target.t
                        )
                        for TRef, qRef, xRef, xDotRef in zip(traj.T, traj.q, traj.x, traj.xDot):
                            q = coppelia.GetJointsPosition()
                            if qRef is None:
                                qRef = robotDh.ikine_LMS(TRef).q
                            if target.controller == 'cart':
                                _, qDotControl = CartesianSpaceController(robotDh, target.Kp, q, TRef, None, qRef)
                            elif target.controller == 'joint':
                                _, qDotControl = JointSpaceController(robotDh, target.Kp, qRef, TRef, q)
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
                                break
                        coppelia.SetJointsTargetVelocity([0,0,0,0,0,0,0]); coppelia.Step(xRef[:3])
                        if hasattr(coppelia, 'Gripper') and target.GripperActuation.close is not None:
                            success = coppelia.Gripper.SetupActuation(target.GripperActuation)
                            if success:
                                start_time = coppelia.sim.getSimulationTime()
                                while coppelia.sim.getSimulationTime() - start_time < 2:
                                    coppelia.Step()
                                    coppelia.Gripper.Actuation()
                        else:
                            success = True
                        if not success:
                            break
                else:
                    coppelia.Step()
        else:
            coppelia.Step()
    else:
        coppelia.Step()

coppelia.Stop()
