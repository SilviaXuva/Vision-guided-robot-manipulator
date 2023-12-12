import numpy as np
from roboticstoolbox.models.URDF.LBR import LBR
from spatialmath import SE3

class LBR_iiwa(LBR):
    """Class that imports a Kuka LBR iiwa 14R 820 URDF model

    ``LBR()`` is a class which imports a Kuka LBR iiwa 14R 820 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> from Models.URDF_LBR_iiwa import LBR_iiwa
        >>> robot = LBR_iiwa()

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """
    
    def __init__(self):
        """Init LBR_iiwa URDF model"""
        deg = np.pi/180
        mm = 1e-3
        flange = 230 * mm

        super().__init__()
        
        self.name = 'LBRiiwa14R820'
        self.qr = np.array([0,0,0, 90*deg, 0, 90*deg,-90*deg])
        self.Tr = self.fkine(self.qr)
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
        self.Tz = self.fkine(self.qz)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        self.q = self.qz
        self.base = SE3.Rz(np.pi)