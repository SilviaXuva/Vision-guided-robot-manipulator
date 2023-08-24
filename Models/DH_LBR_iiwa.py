import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from settings import Settings

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

    def __init__(self, q0: np.ndarray):
        """Init LBR_iiwa custom DH model

        Args:
            q0 (np.ndarray): Initial joints positions [rad].
        """

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

        self.qr = np.array([0,0,0, 90*deg, 0,0,0])
        self.qz = np.zeros(7)
        
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        
        self.q = q0
        
        self.number_joints = self.n
        self.type = 'DH'


if __name__ == "__main__":  # pragma nocover

    robot = LBR_iiwa()
    print(robot)
