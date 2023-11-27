from CoppeliaSim.robotSimulator import RobotSimulator
from Models import DH_LBR_iiwa as LBR_iiwa

robot = LBR_iiwa()
robot.Coppelia = RobotSimulator(robot, drawing = True, gripper = False, vision = True)

robot.Coppelia.start()
robot.Coppelia.step()
robot.Coppelia.Vision.GetImg()

# from CoppeliaSim.vision import VisionNonThreaded
# from zmqRemoteApi import RemoteAPIClient

# client = RemoteAPIClient()
# sim = client.getObject('sim')

# vision = VisionNonThreaded(client, sim)
# sim.startSimulation()
# vision.GetImg()