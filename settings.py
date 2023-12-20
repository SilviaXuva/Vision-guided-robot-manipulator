import cv2
from datetime import datetime
import json
import numpy as np
import os
from spatialmath import SE3
import sys

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
                    Settings.Log(e)
        print('\n'.join(message))
        self.file.write('\n'.join(message) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())

class Settings:
    outputPath = os.path.abspath('.\Outputs')
    startTime = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
    executionPath = fr'{outputPath}\{startTime}'
    os.makedirs(executionPath, exist_ok=True)
    Log = Logger(executionPath)
    logPath = fr'{executionPath}\output.log'
    Ts = 0.05
    Ttol = 5
    t = np.arange(0, Ttol + Ts, Ts)
    Kp = 5 # np.array([5, 5, 5, 5, 5, 5, 5])
    tol = np.array([0.05, 0.055])
    trajectoryType = 'joint'
    trajectorySource = 'rtb'    
    controller = 'joint'

class Cuboids:
    path = './ref_cuboid'
    bodyPath = './ref_body'
    markerPath = './ref_marker{id}'
    maxCreation = 5

class ProximitySensor:
    path = './proximitySensor'

class Conveyor:
    path = './conveyor'

class Camera:
    sensorPath = './camera1'
    distortionCoefficients = None #np.array(distortionCoefficients)
    frameRotation = SE3.Rz(np.pi)

class Aruco:
    dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detectParam = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dict, detectParam)
    length = 0.05
    estimateParam = cv2.aruco.EstimateParameters()
    estimateParam.solvePnPMethod=0
    points = np.array([
        [[-length/2, length/2, 0]],
        [[length/2, length/2, 0]],
        [[length/2, -length/2, 0]],
        [[-length/2, -length/2, 0]]
    ])

class Gripper:
    rotation = SE3.RPY([-np.pi, 0, 0])
    increaseHeight = 0.1