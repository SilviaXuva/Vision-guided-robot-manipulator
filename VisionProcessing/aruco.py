from settings import Settings, Aruco as ArucoMarkers
from Simulators.CoppeliaSim import Camera
from VisionProcessing.colorBasedFilters import MaskRanges, GetGray
from VisionProcessing.guiFeatures import WriteText, DrawingFrame
from Helpers.meanSquareError import MeanSquareError
from Data.pose import Pose

import cv2
import numpy as np
from spatialmath import SE3

class Marker():
    def __init__(
            self, 
            corners:np.ndarray, 
            id: int, 
            color: np.ndarray, 
            rvec = None, tvec = None, 
            objectCameraT = None, objectWorldT = None, 
            T = None, 
            realObjectCameraT = None, realObjectWorldT = None
        ) -> None:
        self.corners = corners
        self.id = id
        self.color = color
        self.rvec = rvec; self.tvec = tvec
        self.objectCameraT = objectCameraT; self.objectWorldT = objectWorldT
        self.T = T
        self.realObjectCameraT = realObjectCameraT; self.realObjectWorldT = realObjectWorldT

class ArucoVision():
    def __init__(self, Camera: Camera) -> None:
        self.Camera = Camera
        self.intrinsicMatrix = Camera.intrinsicMatrix    
        self.distortionCoefficients = Camera.distortionCoefficients  
        self.extrinsicMatrix = Camera.extrinsicMatrix

    def Process(self):
        self.detected = []
        self.PreProcessing()
        self.Detect()
        self.Draw()
        self.DrawPose()
        self.EstimateArucoPose()
        cv2.imshow('Frame', self.Camera.frame)
        cv2.imshow('Processed', self.processedFrame)
        cv2.imshow('Draw', self.drawFrame)
        cv2.waitKey(1)  
    
    def PreProcessing(self):
        self.processedFrame = self.Camera.frame.copy()
        self.processedFrame, _ = MaskRanges(self.processedFrame)
        self.processedFrame = GetGray(self.processedFrame)

    def Detect(self):
        corners, ids, rejectedCandidates = ArucoMarkers.detector.detectMarkers(self.processedFrame)
        
        if len(corners) > 0: # If markers are detected
            for (markerCorners, markerId) in zip(corners, ids):
                min_y = int(min(markerCorners[0][:,1])); max_y = int(max(markerCorners[0][:,1]))
                min_x = int(min(markerCorners[0][:,0])); max_x = int(max(markerCorners[0][:,0]))
                roi = self.Camera.frame[min_y:max_y, min_x:max_x]
                _, colors = MaskRanges(roi)
                markerColor = colors[0] if len(colors) > 0 else None
                self.detected.append(Marker(markerCorners, int(markerId[0]), markerColor))
    
    def Draw(self):
        self.drawFrame = self.Camera.frame.copy()
        corners = [marker.corners for marker in self.detected]
        cv2.aruco.drawDetectedMarkers(self.drawFrame, corners) # Draw a square around the markers
        
        for marker in self.detected:
            min_x = int(min(marker.corners[0][:,1])); max_x = int(max(marker.corners[0][:,1]))
            min_y = int(min(marker.corners[0][:,0])); max_y = int(max(marker.corners[0][:,0]))                
            x = int(max_x + (max_x - min_x)/2)
            y = int(max_y)
            WriteText(self.drawFrame, marker.id, (y, x), thickness=3)

    def DrawPose(self):
        for i, marker in enumerate(self.detected):	
            # Estimate marker pose to camera frame
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                marker.corners, 
                ArucoMarkers.length, 
                self.intrinsicMatrix, 
                self.distortionCoefficients,
                estimateParameters=ArucoMarkers.estimateParam
            )

            self.detected[i].rvec = rvec.flatten()
            self.detected[i].tvec = tvec.flatten()

            DrawingFrame(
                self.drawFrame, 
                self.intrinsicMatrix,
                self.distortionCoefficients,
                rvec,
                tvec,
                ArucoMarkers.length
            )
            
    def GetArucoRealPose(self, markerId: int):
        def TransformCoppeliaMatrix(matrixArray):
            matrix = SE3(np.array(
                np.vstack([
                    np.array(matrixArray).reshape(3,4), 
                    np.array([0,0,0,1])
                ])))
            return matrix
        
        try:
            objectCameraT = TransformCoppeliaMatrix(
                self.Camera.sim.getObjectMatrix(self.Camera.sim.getObject(f'./marker{markerId}'), self.Camera.sim.getObject('./cameraFrame'))
            )   
            objectWorldT = TransformCoppeliaMatrix(
                self.Camera.sim.getObjectMatrix(self.Camera.sim.getObject(f'./marker{markerId}'), self.Camera.sim.handle_world)
            )
            real = [objectCameraT, objectWorldT]
        except:
            real = [None, None]
        return real
    
    def EstimateArucoPose(self):
        for i, marker in enumerate(self.detected):	
            if marker.rvec is not None:
                rmat, _ = cv2.Rodrigues(marker.rvec)

                # Marker pose to camera frame
                objectCameraT = SE3(np.vstack([np.hstack([rmat, np.reshape(marker.tvec,(3,1))]), np.array([0, 0, 0, 1])]))

                # Marker pose to world frame
                objectWorldT = SE3(self.extrinsicMatrix@objectCameraT.A)
                
                T = Pose(
                    x = objectWorldT.t[0], 
                    y = objectWorldT.t[1], 
                    z = objectWorldT.t[2]-(ArucoMarkers.length/2), 
                    rpy = [0, 0, objectWorldT.rpy()[-1]],
                )
                
                real = self.GetArucoRealPose(marker.id)

                self.detected[i].objectCameraT = objectCameraT
                self.detected[i].objectWorldT = objectWorldT
                self.detected[i].T = T
                self.detected[i].realObjectCameraT = real[0]
                self.detected[i].realObjectWorldT = real[1]
                
    def PrintEstimatePose(self, marker: Marker):
        if marker.T is not None:
            Settings.Log(f'========== ID: {marker.id}__Color: {marker.color} ============')
            Settings.Log('---------- Object to camera ------------')
            Settings.Log('---------- r ------------')
            Settings.Log('Estimated:', marker.objectCameraT.rpy())
            Settings.Log('Real:', marker.realObjectCameraT.rpy())
            Settings.Log('Error:', MeanSquareError(marker.objectCameraT.rpy(), marker.realObjectCameraT.rpy()))
            Settings.Log('---------- t ------------')
            Settings.Log('Estimated:', marker.objectCameraT.t)
            Settings.Log('Real:', marker.realObjectCameraT.t)
            Settings.Log('Error:', MeanSquareError(marker.objectCameraT.t, marker.realObjectCameraT.t))               
            Settings.Log('---------- Object to world ------------')
            Settings.Log('---------- r ------------')
            Settings.Log('Estimated:', marker.objectWorldT.rpy())
            Settings.Log('Real:', marker.realObjectWorldT.rpy())
            Settings.Log('Error:', MeanSquareError(marker.objectWorldT.rpy(), marker.realObjectWorldT.rpy()))
            Settings.Log('---------- t ------------')
            Settings.Log('Estimated:', marker.objectWorldT.t)
            Settings.Log('Real:', marker.realObjectWorldT.t)
            Settings.Log('Error:', MeanSquareError(marker.objectWorldT.t, marker.realObjectWorldT.t))
        else:
            Settings.Log(f'Could not estimate marker {marker.color}{marker.id} pose')