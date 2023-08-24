import numpy as np
from roboticstoolbox.models.URDF.LBR import LBR

class LBR_iiwa(LBR):
    """Class that imports a Kuka LBR iiwa 14R 820 URDF model

    ``LBR()`` is a class which imports a Kuka LBR iiwa 14R 820 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> from Models.URDF_LBR_iiwa import LBR_iiwa
        >>> robot = LBR_iiwa()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """
    
    def __init__(self, q0: np.ndarray):
        """Init LBR_iiwa URDF model

        Args:
            q0 (np.ndarray): Initial joints positions [rad].
        """
        
        super().__init__()
        self.q = q0
        
        self.name = 'LBRiiwa14R820'
        self.number_joints = self.n
        self.type = 'URDF'