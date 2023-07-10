import numpy as np
from spatialmath.base import transl, trotz
from roboticstoolbox import DHRobot, RevoluteDH

from Toolbox.EnvironmentPyPlot import Env
from Controller import Controller

from Toolbox.IK import IK

class LBR_iiwa(DHRobot, Controller, IK):
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

    def __init__(self, T_tot = 10):

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
        
        super().__init__(L, name="LBRiiwa14R820", manufacturer="Kuka", tool=tool)

        self.qr = np.array([0,0,0, np.pi/2, 0,0,0])
        self.qz = np.zeros(7)
        self.q0 = np.array([-0.01647629216313362, 0.037338417023420334, 0.0009847808396443725, 0.07846628129482269, -0.0013139393413439393, 0.04261644929647446, 0.017349982634186745])
        self.q = self.q0
        
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("q0", self.q0)
        
        self.number_joints = self.n
        
        self.Ts = 0.05
        self.T_tot = T_tot
        self.t = np.arange(0, self.T_tot + self.Ts, self.Ts)
        
        self.Kp_cart = np.eye(6)*25 # Kp_cart = np.eye(6)*17
        self.Kp_joint = np.eye(7)*35
        self.controller = 'cart'
        
        self.env = Env(self)
        self.q_time = list()
        self.q_control_time = list()

if __name__ == "__main__":  # pragma nocover

    robot = LBR_iiwa()
    print(robot)