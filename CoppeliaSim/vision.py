import cv2
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation

from threading import Thread, Event

from settings import Settings
from VisionProcessing.color_based_filters import removeBackground, maskRanges, getGray, data
from VisionProcessing.filters import getBlur, getThreshold, getCanny, getCornersByGoodFeatures, getCornersByHarris, refineCorners
from VisionProcessing.segmentation import drawEachContourAndCenter, matchShapes
from VisionProcessing.gui_features import writeText
from Data.targets import Target, bins

class VisionNonThreaded:
    def __init__(self, client, sim):
        print('Init Vision...')
        self.client = client
        self.sim = sim
        
        self.vision_sensor_handle = self.sim.getObject(Settings.Camera.sensor_object)
        self.getCameraParameters(Settings.Camera.perspective_angle, Settings.Camera.unit)
        self.getCameraPose()
        cx = self.res_x/2
        cy = self.res_y/2
        fx = cx/np.tan(self.fov_x/2)
        fy = cy/np.tan(self.fov_y/2)
        self.calculated_camera_matrix = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1 ]
        ])
        self.distortion_coefficients = None
        self.T_camera_world = SE3.Trans(self.pos)*SE3.RPY(self.rpy, order='xyz')
        Settings.log('calculated_camera_matrix:', self.calculated_camera_matrix)
        Settings.log('T_camera_world:', np.array(self.T_camera_world))

        self.getArucoRealPose()

    def getCameraParameters(self, perspective_angle = 30, unit = 'deg'):
        self.res_x, self.res_y = self.sim.getVisionSensorRes(self.vision_sensor_handle)
        if unit == 'deg':
            perspective_angle *= np.pi/180
        self.fov_x = self.fov_y = perspective_angle
        self.res_x, self.res_y, self.fov_x, self.fov_y

    def getCameraPose(self):
        self.pos = self.sim.getObjectPosition(self.vision_sensor_handle, self.sim.handle_world)
        self.abg = self.sim.getObjectOrientation(self.vision_sensor_handle, self.sim.handle_world)
        self.ypr = self.sim.alphaBetaGammaToYawPitchRoll(self.abg[0], self.abg[1], self.abg[2])
        self.rpy = [self.ypr[0]-np.pi, self.ypr[1], self.ypr[2]]

    def getArucoRealPose(self):
        def transformCoppeliaMatrix(matrix_array):
            matrix = np.matrix(
                np.vstack([
                    np.array(matrix_array).reshape(3,4), 
                    np.array([0,0,0,1])
                ]))
            return matrix
        properties = {
            't': self.sim.getObjectPosition,
            'r': self.sim.getObjectOrientation,
            'm': self.sim.getObjectMatrix
        }
        system_references = {
            'world': self.sim.handle_world,
            'camera': self.vision_sensor_handle
        }
        self.coppelia_properties = {}

        for i in range(1, 100):
            try:
                self.coppelia_properties[f'marker{i}'] = self.sim.getObject(f'./marker{i}')
                for sys_ref in system_references.keys():
                    for prop in properties.keys():
                        if prop == 'm':
                            self.coppelia_properties[f'marker{i}_{sys_ref}_{prop}'] = transformCoppeliaMatrix(properties[prop](self.coppelia_properties[f'marker{i}'], system_references[sys_ref]))
                        else:
                            self.coppelia_properties[f'marker{i}_{sys_ref}_{prop}'] = np.array(properties[prop](self.coppelia_properties[f'marker{i}'], system_references[sys_ref]))
            except:
                self.coppelia_properties['camera'] = self.vision_sensor_handle
                system_references = {
                    'world': self.sim.handle_world
                }
                for i in range(1, i):
                    system_references[f'marker{i}'] = self.coppelia_properties[f'marker{i}']
                for sys_ref in system_references.keys():
                    for prop in properties.keys():
                        if prop == 'm':
                            self.coppelia_properties[f'camera_{sys_ref}_{prop}'] = transformCoppeliaMatrix(properties[prop](self.coppelia_properties['camera'], system_references[sys_ref]))
                        else:
                            self.coppelia_properties[f'camera_{sys_ref}_{prop}'] = np.array(properties[prop](self.coppelia_properties['camera'], system_references[sys_ref]))
                break
        return self.coppelia_properties

    def getImg(self):
        frame, resX, resY = self.sim.getVisionSensorCharImage(self.vision_sensor_handle)
        frame = np.frombuffer(frame, dtype=np.uint8).reshape(resY, resX, 3)

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        self.frame = cv2.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)
        self.output = self.frame.copy()
        self.draw = self.frame.copy()
    
    def showImg(self):
        cv2.imshow('Draw', self.draw)
        cv2.imshow('Output', self.output)
        cv2.imshow('Frame', self.frame)
        cv2.waitKey(1)

    def preProcessing(self):
        # self.output = removeBackground(self.output)
        self.output,_ = maskRanges(self.output)
        self.output = getGray(self.output)

    def detectAruco(self):
        self.aruco_corners, ids, rejectedCandidates = Settings.Aruco.detector.detectMarkers(self.output)
        self.aruco_markers = list()
        # If markers are detected
        if len(self.aruco_corners) > 0:
            for (marker_corners, marker_ID) in zip(self.aruco_corners, ids):
                roi = self.frame[int(marker_corners[0][0][1]):int(marker_corners[0][2][1]), int(marker_corners[0][0][0]):int(marker_corners[0][1][0])]
                _,colors = maskRanges(roi)
                self.aruco_markers.append(ArucoMarker(marker_corners, marker_ID[0], colors[0]))

    def drawAruco(self):
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(self.draw, self.aruco_corners)
        # If markers are detected
        if len(self.aruco_markers) > 0:
            for i, marker in enumerate(self.aruco_markers):	
                writeText(self.draw, marker.id, (int(marker.corners[0][0][0]), int(marker.corners[0][0][1])))

    def drawArucoPose(self):
        # If markers are detected
        if len(self.aruco_markers) > 0:
            for i, marker in enumerate(self.aruco_markers):	
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                # Aruco to camera frame			
                rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(marker.corners, Settings.Aruco.aruco_lenght, self.calculated_camera_matrix, self.distortion_coefficients)
                tvec = tvec[0].transpose()
                self.aruco_markers[i].rvec = rvec
                self.aruco_markers[i].tvec = tvec

                # Draw Axis
                cv2.drawFrameAxes(self.draw, self.calculated_camera_matrix, self.distortion_coefficients, rvec, tvec, Settings.Aruco.aruco_lenght)

    def estimateArucoPose(self):
        # If markers are detected
        if len(self.aruco_markers) > 0:
            for i, marker in enumerate(self.aruco_markers):	
                rmat, _ = cv2.Rodrigues(marker.rvec)

                # Matriz de transformação da câmera para o sistema do marcador
                T_object_camera = np.vstack([np.hstack([rmat.transpose(), marker.tvec]), np.array([0, 0, 0, 1])])

                # Matriz de transformação do marcador para o sistema global
                T_object_world = np.dot(self.T_camera_world, T_object_camera)
                object_world_r = T_object_world[:3, :3]
                object_world_t = T_object_world[:3, 3]
                
                Settings.log((f'========== ID: {marker.id} ============'))
                Settings.log('---------- Object to world ------------')
                Settings.log('---------- r ------------')
                Settings.log('Estimated:', Rotation.from_matrix(object_world_r).as_euler('xyz'))
                Settings.log('Real:', self.coppelia_properties[f'marker{marker.id}_world_r'])
                Settings.log('---------- t ------------')
                Settings.log('Estimated:', object_world_t)
                Settings.log('Real:', self.coppelia_properties[f'marker{marker.id}_world_t'])

                self.aruco_markers[i].object_world_r = object_world_r
                self.aruco_markers[i].object_world_t = object_world_t

    def getTargets(self):
        pick_and_place = list()
        if len(self.aruco_markers) > 0:
            for marker in self.aruco_markers:
                pick_and_place.append([
                    Target(
                        marker.object_world_t[0], marker.object_world_t[1], marker.object_world_t[2], -np.pi, 0, np.pi/2, 'close', f'./{marker.color}'
                    ),
                    bins[marker.color]
                ])
        return pick_and_place

class ArucoMarker():
    def __init__(self, corners, id, color) -> None:
        self.corners = corners
        self.id = id
        self.color = color

class VisionThreaded(Thread):
    def __init__(self, client, sim):
        Thread.__init__(self)
        self.client = client
        self.sim = sim
        
        self.vision_sensor_handle = self.sim.getObject('/Vision_sensor')
        
        self.stop = Event()

    def run(self):
        while True:
            img, res_x, res_y = self.sim.getVisionSensorCharImage(self.vision_sensor_handle)
            try:
                img = np.frombuffer(img, dtype=np.uint8).reshape(res_y, res_x, 3)

                # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
                # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
                # and color format is RGB triplets, whereas OpenCV uses BGR:
                # img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
                img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 1)
                
            except:
                pass
            
            if self.stop.is_set():
                break
