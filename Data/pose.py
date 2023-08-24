from spatialmath import SE3

class Pose:
    def __init__(self, x, y, z, rx, ry, rz) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        
        self.T = SE3.Trans(
            self.x, self.y, self.z
        )*SE3.Rx(self.rx)*SE3.Ry(self.ry)*SE3.Rz(self.rz)
