import cv2
from datetime import datetime
import numpy as np
import os
import sys
import json
from spatialmath import SE3, SO3

class Logger(object):
    def __init__(self, folder):
        self.terminal = sys.stdout
        self.file = open(fr'{folder}\output.log', 'w', encoding = 'utf-8')
   
    def __call__(self, *message):
        message = list(message)
        for i, msg in enumerate(message):
            if isinstance(msg, np.ndarray):
                message[i] = np.array2string(msg, formatter={'float_kind':lambda x: "%.5f" % x})
            elif isinstance(msg, list):
                message[i] = ', '.join([str(m) for m in msg])
            elif isinstance(msg, dict):
                msg = {(k):(np.array2string(v, formatter={'float_kind':lambda x: "%.5f" % x}) if isinstance(v, np.ndarray) else v) for k,v in msg.items()}
                message[i] = json.dumps(msg, indent=4)
            else:
                try:
                    message[i] = str(msg)
                except Exception as e:
                    Settings.log(e)
        self.terminal.write('\n'.join(message) + '\n')
        self.file.write('\n'.join(message) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())

class Camera:
    def __init__(self, sensor_object='./camera1', distortion_coefficients=None, frame_rotation = SE3.Rz(np.pi)) -> None:
        self.sensor_object = sensor_object
        if distortion_coefficients is None:
            self.distortion_coefficients = None
        else:
            self.distortion_coefficients = np.array(distortion_coefficients)
        self.frame_rotation = frame_rotation

class Aruco:
    def __init__(self, aruco_dict=cv2.aruco.DICT_6X6_250, length=0.05) -> None:
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        detect_param = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, detect_param)
        self.length = length
        self.estimate_param = cv2.aruco.EstimateParameters()
        self.estimate_param.solvePnPMethod=0
        self.marker_points = np.array([
            [[-length/2, length/2, 0]],
            [[length/2, length/2, 0]],
            [[length/2, -length/2, 0]],
            [[-length/2, -length/2, 0]]
        ])

class Settings:
    T_tol = 10
    Ts = 0.05
    output_path = os.path.abspath('.\Outputs')
    start_time = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
    execution_path = fr'{output_path}\{start_time}'
    os.makedirs(execution_path, exist_ok=True)
    log = Logger(execution_path)
    log_path = fr'{execution_path}\output.log'
    pre_processing_parameters_path = ''

    Camera = Camera(sensor_object='/camera1',distortion_coefficients=None, frame_rotation = SE3.Rz(np.pi))
    Aruco = Aruco(aruco_dict=cv2.aruco.DICT_6X6_250, length=0.05)
    gripper_rotation = SE3.RPY([-3.141047184, -0.001486914712, 0])
    target_increase_height = 0.15
    plot = True
