import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from Data.targets import Target

class LBR_iiwa(DHRobot):
    """Class that models a LBR iiwa 14R 820 manipulator

    ``LBR_iiwa()`` is a class which models a Kuka LBR iiwa 14R 820 robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> from Models.DH_LBR_iiwa import LBR_iiwa
        >>> robot = LBR_iiwa(np.array([0,0,0,0,0,0,0,0]))
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration

    .. note:: SI units are used.

    .. codeauthor:: Peter Corke
    """  # noqa

    def __init__(self):
        """Init LBR_iiwa custom DH model"""

        deg = np.pi/180
        mm = 1e-3
        flange = 220 * mm

        # This Kuka model is defined using modified
        # Denavit-Hartenberg parameters
        L = [
            RevoluteDH( d = 0.360 , a = 0.0000, alpha = 90*deg ,  qlim = [-180*deg, 180*deg] ),
            RevoluteDH( d = 0.000 , a = 0.0000, alpha = -90*deg,  qlim = [-120*deg, 120*deg] ),
            RevoluteDH( d = 0.420 , a = 0.0000, alpha = -90*deg,  qlim = [-180*deg, 180*deg] ),
            RevoluteDH( d = 0.000 , a = 0.0000, alpha = 90*deg ,  qlim = [-120*deg, 120*deg] ),
            RevoluteDH( d = 0.400 , a = 0.0000, alpha = 90*deg ,  qlim = [-180*deg, 180*deg] ),
            RevoluteDH( d = 0.000 , a = 0.0000, alpha = -90*deg,  qlim = [-130*deg, 130*deg] ),
            RevoluteDH( d = flange, a = 0.0000, alpha = 0.0000 ,  qlim = [-180*deg, 180*deg] ),
        ]
        
        self.name = 'LBRiiwa14R820'
        DHRobot.__init__(self, L, name=self.name, manufacturer="Kuka")

        self.qr = np.array([0,0,0, 90*deg, 0, 90*deg,-90*deg])
        Tr = self.fkine(self.qr)
        self.Tr = Target(Tr.t[0], Tr.t[1], Tr.t[2], Tr.rpy(order='xyz')[0], Tr.rpy(order='xyz')[1], Tr.rpy(order='xyz')[2], None, None)
        self.qz = np.array(
            [
                -0.01647629216313362, 
                0.037338417023420334, 
                0.0009847808396443725, 
                0.07846628129482269, 
                -0.0013139393413439393, 
                0.04261644929647446, 
                0.017349982634186745
            ]
        )
        Tz = self.fkine(self.qz)
        self.Tz = Target(Tz.t[0], Tz.t[1], Tz.t[2], Tz.rpy(order='xyz')[0], Tz.rpy(order='xyz')[1], Tz.rpy(order='xyz')[2], None, None)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        
        self.number_joints = self.n
        self.type = 'DH'


if __name__ == "__main__":  # pragma nocover

    robot = LBR_iiwa()
    print(robot)
