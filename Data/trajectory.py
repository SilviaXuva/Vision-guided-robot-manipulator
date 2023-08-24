import roboticstoolbox as rtb
from settings import Settings
from spatialmath import SE3

class Trajectory:
    def __init__(self, robot, T1, type) -> None:
        self.q0 = robot.Coppelia.getJointsPosition()
        self.T0 = robot.fkine(self.q0)
        self.T1 = T1
        self.q1 = robot.ikine_LMS(self.T1).q
        if type == 'cart':
            self.ctraj = rtb.ctraj(self.T0, self.T1, Settings.t) # Calculate reference cartesian/end-effector trajectory
            self.ref = [SE3(pose) for pose in self.ctraj.A]  # Transform each pose into SE3]
            self.q_ref = [robot.ikine_LMS(pose).q for pose in self.ctraj.A]
        elif type == 'joint':
            self.jtraj = rtb.jtraj(self.q0, self.q1, Settings.t)  # Calculate reference joints trajectory
            self.ref = [SE3(robot.fkine(q)) for q in self.jtraj.q]  # Transform each joint position into SE3 through forward kinematics
            self.q_ref = [q for q in self.jtraj.q]
        self.q = list()
