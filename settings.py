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
            self.Kp_cart = np.concatenate(
            [
                np.eye(6)[:3]*Kp_trans, 
                np.eye(6)[3:]*Kp_rot
            ]
        ) 
        elif type == 'joint':
            self.Kp_joint = np.diag(Kp)

class Tolerance:
    def __init__(self, type, tol) -> None:
        self.type = type
        if type == 'cart':
            self.tol_trans = tol[0]
            self.tol_rot = tol[1]
        elif type == 'joint':
            for i in range(len(tol)):
                setattr(self, f'tol_joint_{i}', tol[i])

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
    
    Trajectory = Trajectory('cart', 'rtb')
    # Controller = Controller(None)
    # Controller = Controller('cart', [8,6])
    Controller = Controller('joint', [1,1,1,1,1,1,1])
    Tolerance = Tolerance('cart', [0.005, 1])

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
    execution_path = fr'{output_path}\{start_time}'
    os.makedirs(execution_path, exist_ok=True)
    log = Logger(execution_path)
    log_path = fr'{execution_path}\output.log'
    f = open(log_path, 'w')
    f.write(f'''
Trajectory
type = {Trajectory.type}
source = {Trajectory.source}
Controller
type = {Controller.type}'''
    )
    f.close()
