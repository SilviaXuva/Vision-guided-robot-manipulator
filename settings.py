import cv2
from datetime import datetime
import numpy as np
import os
import sys
import json

class Logger(object):
    def __init__(self, folder):
        self.terminal = sys.stdout
        self.file = open(fr'{folder}\output.log', 'w', encoding = 'utf-8')
   
    def __call__(self, *message):
        message = list(message)
        for i, msg in enumerate(message):
            if isinstance(msg, np.ndarray):
                message[i] = np.array2string(msg, formatter={'float_kind':lambda x: "%.2f" % x})
            elif isinstance(msg, list):
                message[i] = ', '.join([str(m) for m in msg])
            elif isinstance(msg, dict):
                message[i] = json.dumps(msg, indent=4)
        self.terminal.write('\n'.join(message) + '\n')
        self.file.write('\n'.join(message) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())

class Controller:
    def __init__(self, type, Kp = []) -> None:
        self.type = type
        if type == 'cart':
            Kp_trans = Kp[0]
            Kp_rot = Kp[1]
            self.Kp = np.concatenate(
            [
                np.eye(6)[:3]*Kp_trans, 
                np.eye(6)[3:]*Kp_rot
            ]
        ) 
        elif type == 'joint':
            self.Kp = np.diag(Kp)

class Tolerance:
    def __init__(self, tol_trans, tol_rot) -> None:
        self.tol_trans = tol_trans
        self.tol_rot = tol_rot

class Trajectory:
    def __init__(self, type, source) -> None:
        self.type = type
        self.source = source

class Camera:
    def __init__(self, sensor_object='./camera1', perspective_angle=30, unit='deg', distortion_coefficients=None) -> None:
        self.sensor_object = sensor_object
        self.perspective_angle = perspective_angle
        self.unit = unit
        self.distortion_coefficients = distortion_coefficients

class Aruco:
    def __init__(self, aruco_dict=cv2.aruco.DICT_6X6_250, aruco_lenght=0.05) -> None:
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        self.aruco_lenght = aruco_lenght

class Settings:
    Ts = 0.05
    T_tot = 10
    t = np.arange(0, T_tot + Ts, Ts)
    Tolerance = Tolerance(tol_trans = 0.005, tol_rot = 1)

    # Controller = Controller(None)
    # Trajectory = Trajectory('cart', 'custom')
    # Controller = Controller(None)
    # Trajectory = Trajectory('cart', 'rtb')
    # Controller = Controller(None)
    # Trajectory = Trajectory('joint', 'custom')
    # Controller = Controller(None)
    # Trajectory = Trajectory('joint', 'rtb')

    # Controller = Controller('cart', [8,6])
    # Trajectory = Trajectory('cart', 'custom')
    # Controller = Controller('cart', [8,6])
    # Trajectory = Trajectory('cart', 'rtb')
    # Controller = Controller('cart', [8,6])
    # Trajectory = Trajectory('joint', 'custom')
    # Controller = Controller('cart', [8,6])
    # Trajectory = Trajectory('joint', 'rtb')

    # Controller = Controller('joint', [1,1,1,1,1,1,1])
    # Trajectory = Trajectory('cart', 'custom')
    # Controller = Controller('joint', [1,1,1,1,1,1,1])
    # Trajectory = Trajectory('cart', 'rtb')
    # Controller = Controller('joint', [5,5,5,5,5,5,5])
    # Trajectory = Trajectory('joint', 'custom')
    Controller = Controller('joint', [5,5,5,5,5,5,5])
    Trajectory = Trajectory('joint', 'rtb')

    plot = True
    
    q0 = np.array(
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

    Camera = Camera(sensor_object='/camera1', perspective_angle=30, unit='deg')
    Aruco = Aruco(aruco_dict=cv2.aruco.DICT_6X6_250, aruco_lenght=0.05)
    pre_processing_parameters_path = os.path.abspath(r'.\VisionProcessing\pre_processing_parameters.npz')

    output_path = os.path.abspath('.\Outputs')
    start_time = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
    execution_path = fr'{output_path}\{str(Controller.type).title()} Controller\{Trajectory.type.title()} {Trajectory.source.title()} Trajectory\{start_time}'
    os.makedirs(execution_path, exist_ok=True)
    log = Logger(execution_path)
    log_path = fr'{execution_path}\output.log'
    f = open(log_path, 'w')
    f.write(f'''
Trajectory
type = {Trajectory.type}
source = {Trajectory.source}
Controller
type = {Controller.type if Controller.type is not None else 'None'}'''
    )
    f.close()
