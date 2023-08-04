import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl, trotz

from Model.Settings import Settings
from Model.Controller import Controller

from Coppelia.APIFunctions import Coppelia
from Coppelia.Gripper import Gripper_ChildScript
from Toolbox.PyPlotFunctions import PyPlotEnv

class LBR_iiwa(DHRobot, Settings, Controller):
    """
    Class that models a LBR iiwa 14R 820 manipulator

    ``LBR_iiwa()`` is a class which models a Kuka LBR iiwa 14R 820 robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = models.DH.LBR_iiwa()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration

    .. note:: SI units are used.

    .. codeauthor:: Peter Corke
    """  # noqa

    def __init__(self):

        # deg = np.pi/180
        mm = 1e-3
        tool_offset = (19) * mm

        flange = 210 * mm

        # This Kuka model is defined using modified
        # Denavit-Hartenberg parameters
        L = [
            RevoluteDH(d=0.360,  a = 0, alpha=np.pi/2,   qlim=[      -np.pi,           np.pi]    ),
            RevoluteDH(d=0.000,  a = 0, alpha=-np.pi/2,  qlim=[  -2*np.pi/3,       2*np.pi/3]    ),
            RevoluteDH(d=0.420,  a = 0, alpha=-np.pi/2,  qlim=[      -np.pi,           np.pi]    ),
            RevoluteDH(d=0.000,  a = 0, alpha=np.pi/2,   qlim=[  -2*np.pi/3,       2*np.pi/3]    ),
            RevoluteDH(d=0.400,  a = 0, alpha=np.pi/2,   qlim=[      -np.pi,           np.pi]    ),
            RevoluteDH(d=0.000,  a = 0, alpha=-np.pi/2,  qlim=[-13*np.pi/18,     13*np.pi/18]    ),
            RevoluteDH(d=flange, a = 0, alpha=0,         qlim=[      -np.pi,           np.pi]    ),
            # RevoluteDH(d=0.229, a = 0, alpha=0,                                     )
        ]

        tool = transl(0, 0, tool_offset)
        
        name = 'LBRiiwa14R820'
        DHRobot.__init__(self, L, name=name, manufacturer="Kuka", tool=tool)

        self.qr = np.array([0,0,0, np.pi/2, 0,0,0])
        self.qz = np.zeros(7)
        
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        
        Settings.__init__(self)
        
        self.instance_pyplot = PyPlotEnv(self) if self.toolbox else None
        if self.coppelia:
            self.instance_coppelia = Coppelia(self)
            self.Gripper = Gripper_ChildScript()
        else:
            self.instance_coppelia = None

    def startSimulation(self):
        if self.instance_pyplot:
            self.instance_pyplot.startSimulation()
        if self.instance_coppelia:
            self.instance_coppelia.startSimulation()

    def stopSimulation(self):
        if self.instance_pyplot:
            self.instance_pyplot.stopSimulation()
        if self.instance_coppelia:
            self.instance_coppelia.stopSimulation()

    def getJointPosition(self):
        if self.instance_coppelia:
            q = self.instance_coppelia.getJointPosition()
        else:
            if self.instance_pyplot:
                q = self.instance_pyplot.getJointPosition()
        return q

    def setJointTargetVelocity(self, vel):
        if self.instance_pyplot:
            self.instance_pyplot.setJointTargetVelocity(vel)
        if self.instance_coppelia:
            self.instance_coppelia.setJointTargetVelocity(vel)
        
    def setJointTargetPosition(self, pos):
        if self.instance_pyplot:
            self.instance_pyplot.setJointTargetPosition(pos)
        if self.instance_coppelia:
            self.instance_coppelia.setJointTargetPosition(pos)

    # called when an attribute is not found:
    def __getattr__(self, name):
        try:
            result = self.instance_pyplot.__getattribute__(name)
        except:
            result = self.NoneFunction
        return result
    
    def NoneFunction(*args, **kwargs):
        pass

if __name__ == "__main__":  # pragma nocover

    robot = LBR_iiwa()
    print(robot)
