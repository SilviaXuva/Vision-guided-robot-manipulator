import time
from Toolbox.LBR_iiwa_DH import LBR_iiwa
from ZMQ.Coppelia import Coppelia
from ZMQ.Gripper import Gripper_ChildScript

class Coppelia_LBR_iiwa(LBR_iiwa, Coppelia):
    def __init__(self, T_tot, gripper = True):
        LBR_iiwa.__init__(self, T_tot=T_tot)
        Coppelia.__init__(self)
        self.stopSimulation()
        time.sleep(2)
        self.model = 'LBRiiwa14R820'
        self.handle = self.getRobotHandle()
        self.joints = self.getJoints()
        
        if gripper:
            self.Gripper = Gripper_ChildScript()