from settings import Settings
from VisionProcessing.colorBasedFilters import maskRanges, getGray
from VisionProcessing.guiFeatures import writeText, drawingFrame
from Data.meanSquareError import meanSquareError
from Data.targets import bins, Target
from CoppeliaSim.gripper import Actuation

from spatialmath import SE3
import numpy as np
import cv2
from collections import namedtuple
from threading import Thread, Event
import math

class VisionNonThreaded:
    def __init__(self, client, sim):
        Settings.log('Init Vision...')
        self.client = client
        self.sim = sim
        
        try:
            self.proximity_handle = self.sim.getObjectHandle('./ref_proximitySensor')
            self.conveyor_handle = self.sim.getObjectHandle('./conveyor')
        except:
            self.proximity_handle = None
            self.conveyor_handle = None

        self.vision_sensor_handle = self.sim.getObject(Settings.Camera.sensor_object)
        self.getCameraParameters()
        self.getIntrinsicMatrix()
        Settings.log('camera_intrinsic:', self.camera_intrinsic)
        Settings.log('distortion_coefficients:', self.distortion_coefficients)        
        self.getExtrinsicMatrix()
        Settings.log('camera_extrinsic:', self.camera_extrinsic)

    def getCameraParameters(self):
        self.res_x, self.res_y = self.sim.getVisionSensorRes(self.vision_sensor_handle)
        self.perspective_angle = self.sim.getObjectFloatParam(self.vision_sensor_handle, self.sim.visionfloatparam_perspective_angle)
        self.fov_x = self.fov_y = self.perspective_angle
        self.res_x, self.res_y, self.fov_x, self.fov_y

    def getExtrinsicMatrix(self):
        pos = self.sim.getObjectPosition(self.vision_sensor_handle, self.sim.handle_world)
        abg = self.sim.getObjectOrientation(self.vision_sensor_handle, self.sim.handle_world)
        ypr = self.sim.alphaBetaGammaToYawPitchRoll(abg[0], abg[1], abg[2])
        rpy = [ypr[0], ypr[1], ypr[2]]
        camera_world_T = SE3.Trans(pos)*SE3.RPY(rpy, order='zyx')
        Settings.log('camera_world_T:', camera_world_T.A)
        self.camera_extrinsic = (camera_world_T*Settings.Camera.frame_rotation).A

    def getIntrinsicMatrix(self):
        cx = self.res_x/2
        cy = self.res_y/2
        fx = cx/np.tan(self.fov_x/2)
        fy = cy/np.tan(self.fov_y/2)
        self.camera_intrinsic = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1 ]
        ])
        self.distortion_coefficients = Settings.Camera.distortion_coefficients

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
        try:
            cv2.imshow('Draw', self.draw)
        except Exception as e:
            Settings.log('Show drawing image error', e)
        try:
            cv2.imshow('Processed', self.output)
        except Exception as e:
            Settings.log('Show processed image error', e)            
        try:
            cv2.imshow('Frame', self.frame)
        except Exception as e:
            Settings.log('Show frame image error', e)
        cv2.waitKey(1)

    def preProcessing(self):
        # self.output = removeBackground(self.output)
        self.output,_ = maskRanges(self.output)
        self.output = getGray(self.output)

    def detectAruco(self):
        self.aruco_markers = list()
        self.aruco_corners, ids, rejectedCandidates = Settings.Aruco.detector.detectMarkers(self.output)
        
        if len(self.aruco_corners) > 0: # If markers are detected
            for (marker_corners, marker_ID) in zip(self.aruco_corners, ids):
                min_y = int(min(marker_corners[0][:,1])); max_y = int(max(marker_corners[0][:,1]))
                min_x = int(min(marker_corners[0][:,0])); max_x = int(max(marker_corners[0][:,0]))
                roi = self.frame[min_y:max_y, min_x:max_x]
                _, colors = maskRanges(roi)
                marker_color = colors[0] if len(colors) > 0 else None
                self.aruco_markers.append(ArucoMarker(marker_corners, int(marker_ID[0]), marker_color))

    def drawAruco(self):
        cv2.aruco.drawDetectedMarkers(self.draw, self.aruco_corners) # Draw a square around the markers
        
        if len(self.aruco_markers) > 0: # If markers are detected
            for marker in self.aruco_markers:
                min_x = int(min(marker.corners[0][:,1])); max_x = int(max(marker.corners[0][:,1]))
                min_y = int(min(marker.corners[0][:,0])); max_y = int(max(marker.corners[0][:,0]))                
                x = int(max_x + (max_x - min_x)/2)
                y = int(max_y)
                writeText(self.draw, marker.id, (y, x), thickness=3)

    def drawArucoPose(self):
        if len(self.aruco_markers) > 0: # If markers are detected
            for i, marker in enumerate(self.aruco_markers):	
                # Estimate marker pose to camera frame
                rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(
                    marker.corners, 
                    Settings.Aruco.length, 
                    self.camera_intrinsic, 
                    self.distortion_coefficients,
                    estimateParameters=Settings.Aruco.estimate_param
                )

                self.aruco_markers[i].rvec = rvec.flatten()
                self.aruco_markers[i].tvec = tvec.flatten()

                drawingFrame(
                    self.draw, 
                    self.camera_intrinsic,
                    self.distortion_coefficients,
                    rvec,
                    tvec,
                    Settings.Aruco.length
                )

    def getArucoRealPose(self, marker_id: int):
        RealPose = namedtuple('RealPose', ['camera_T', 'world_T'])
        def transformCoppeliaMatrix(matrix_array):
            matrix = SE3(np.array(
                np.vstack([
                    np.array(matrix_array).reshape(3,4), 
                    np.array([0,0,0,1])
                ])))
            return matrix
        
        try:
            marker_camera_T = transformCoppeliaMatrix(
                self.sim.getObjectMatrix(self.sim.getObject(f'./marker{marker_id}'), self.sim.getObject('./cameraFrame'))
            )   
            marker_world_T = transformCoppeliaMatrix(
                self.sim.getObjectMatrix(self.sim.getObject(f'./marker{marker_id}'), self.sim.handle_world)
            )
            real_pose = RealPose(marker_camera_T, marker_world_T)
        except:
            real_pose = RealPose(None, None)
        return real_pose
    
    def estimateArucoPose(self):
        if self.proximity_handle is None or (self.sim.readProximitySensor(self.proximity_handle)[0] == 1 and math.isclose(self.sim.readCustomTableData(self.conveyor_handle,'__state__')['vel'], 0, abs_tol=0.0001)):
            marker = self.aruco_markers[0]
            if len(self.aruco_markers) > 0 and hasattr(marker, 'rvec'): # If markers are detected
                rmat, _ = cv2.Rodrigues(marker.rvec)

                # Marker pose to camera frame
                object_camera_T = SE3(np.vstack([np.hstack([rmat, np.reshape(marker.tvec,(3,1))]), np.array([0, 0, 0, 1])]))

                # Marker pose to world frame
                object_world_T = SE3(self.camera_extrinsic@object_camera_T.A)
                
                self.aruco_markers[0].object_world_T = object_world_T

                real_pose = self.getArucoRealPose(marker.id)
                Settings.log(f'========== ID: {marker.id}__Color: {marker.color} ============')
                Settings.log('---------- Object to camera ------------')
                Settings.log('---------- r ------------')
                Settings.log('Estimated:', object_camera_T.rpy())
                Settings.log('Real:', real_pose.camera_T.rpy())
                Settings.log('Error:', meanSquareError(object_camera_T.rpy(), real_pose.camera_T.rpy()))
                Settings.log('---------- t ------------')
                Settings.log('Estimated:', object_camera_T.t)
                Settings.log('Real:', real_pose.camera_T.t)
                Settings.log('Error:', meanSquareError(object_camera_T.t, real_pose.camera_T.t))               
                Settings.log('---------- Object to world ------------')
                Settings.log('---------- r ------------')
                Settings.log('Estimated:', object_world_T.rpy())
                Settings.log('Real:', real_pose.world_T.rpy())
                Settings.log('Error:', meanSquareError(object_world_T.rpy(), real_pose.world_T.rpy()))
                Settings.log('---------- t ------------')
                Settings.log('Estimated:', object_world_T.t)
                Settings.log('Real:', real_pose.world_T.t)
                Settings.log('Error:', meanSquareError(object_world_T.t, real_pose.world_T.t))

    def getTarget(self):
        if self.proximity_handle is None or (self.sim.readProximitySensor(self.proximity_handle)[0] == 1 and math.isclose(self.sim.readCustomTableData(self.conveyor_handle,'__state__')['vel'], 0, abs_tol=0.0001)):
            marker = self.aruco_markers[0]
            if len(self.aruco_markers) > 0 and hasattr(marker, 'object_world_T'):
                bin = bins[marker.color]
                bin.Gripper_actuation = Actuation(
                    actuation = 'open', 
                    shape_path = f'./{marker.color}{marker.id}'
                )
                pick_and_place = [
                    Target(
                        x = marker.object_world_T.t[0], 
                        y = marker.object_world_T.t[1], 
                        z = marker.object_world_T.t[2] + Settings.target_increase_height, 
                        rpy = marker.object_world_T.rpy()
                    ),
                    Target(
                        x = marker.object_world_T.t[0], 
                        y = marker.object_world_T.t[1], 
                        z = marker.object_world_T.t[2], 
                        rpy = marker.object_world_T.rpy(),
                        gripper_actuation = Actuation(
                            actuation = 'close', 
                            shape_path = f'./{marker.color}{marker.id}'
                        )
                    ),
                    bin
                ]
            return pick_and_place
        else:
            return [None, None, None]

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
