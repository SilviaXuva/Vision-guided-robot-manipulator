from settings import Settings

class Sim:
    def __init__(self, robot) -> None:
        self.robot = robot
    
    def Step(self):
        pass

    def Start(self, sim):
        Settings.Log(f'{sim} simulation started')

    def Stop(self, sim):
        Settings.Log(f'{sim} simulation stopped')

    def GetJointsPosition(self):
        pass

    def SetJointsTargetVelocity(self):
        pass