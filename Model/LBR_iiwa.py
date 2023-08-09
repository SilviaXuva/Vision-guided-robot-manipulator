import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl, trotz

from Model.Settings import Settings
from Model.Controller import Controller
from Simulators import Simulators

class LBR_iiwa(DHRobot, Settings, Controller, Simulators):
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
        
        self.name = 'LBRiiwa14R820'
        DHRobot.__init__(self, L, name=self.name, manufacturer="Kuka", tool=tool)

        self.qr = np.array([0,0,0, np.pi/2, 0,0,0])
        self.qz = np.zeros(7)
        
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        
        Settings.__init__(self)
        Controller.__init__(self)
        Simulators.__init__(self)

if __name__ == "__main__":  # pragma nocover

    robot = LBR_iiwa()
    print(robot)
