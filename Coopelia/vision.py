import cv2
import numpy as np
from MachineVision.filters import maskHsv, hsvFloor, getGray, getThreshold, getShapes
from threading import Thread, Event

class VisionNonThreaded:
    def __init__(self, client, sim):
        self.client = client
        self.sim = sim
        
        self.vision_sensor_handle = self.sim.getObject('/camera1')
    
    def GetImg(self):
        frame, resX, resY = self.sim.getVisionSensorCharImage(self.vision_sensor_handle)
        frame = np.frombuffer(frame, dtype=np.uint8).reshape(resY, resX, 3)

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        frame = cv2.flip(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), 0)

        _, floorOutCropped = maskHsv(
            frame, 
            cv2.cvtColor(frame, cv2.COLOR_RGB2HSV), 
            hsvFloor.low,
            hsvFloor.high,
            invert=True
        )
        gray = getGray(floorOutCropped)
        thresh = getThreshold(gray)
        getShapes(frame, thresh)
        
        cv2.imshow('Output', frame)
        cv2.waitKey(1)

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

                _, floorOutCropped = maskHsv(
                    img, 
                    cv2.cvtColor(img, cv2.COLOR_RGB2HSV), 
                    hsvFloor.low,
                    hsvFloor.high,
                    invert=True
                )
                cv2.imshow('Original', img)
                cv2.imshow('Without floor', floorOutCropped)
                cv2.imshow('', getShapes(img, floorOutCropped))
                cv2.waitKey(1)
                
            except:
                pass
            
            if self.stop.is_set():
                break
