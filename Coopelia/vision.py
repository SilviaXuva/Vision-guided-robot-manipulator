import cv2
import numpy as np
from ProcessVision.mask import hsv_floor, maskHsv
from ProcessVision.shapes import getShapes
from threading import Thread, Event

class VisionNonThreaded:
    def __init__(self, client, sim):
        self.client = client
        self.sim = sim
        
        self.vision_sensor_handle = self.sim.getObject('/camera1')
    
    def GetImg(self):
        img, res_x, res_y = self.sim.getVisionSensorCharImage(self.vision_sensor_handle)
        try:
            img = np.frombuffer(img, dtype=np.uint8).reshape(res_y, res_x, 3)

            # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
            # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
            # and color format is RGB triplets, whereas OpenCV uses BGR:
            # img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
            img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

            _, floor_out_cropped = maskHsv(
                img, 
                cv2.cvtColor(img, cv2.COLOR_RGB2HSV), 
                hsv_floor.low,
                hsv_floor.high,
                invert=True
            )
            # cv2.imshow('Original', img)
            # cv2.imshow('Without floor', floor_out_cropped)
            cv2.imshow('', getShapes(img, floor_out_cropped))
            cv2.waitKey(1)

        except Exception as e:
            print(e)
            pass

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

                _, floor_out_cropped = maskHsv(
                    img, 
                    cv2.cvtColor(img, cv2.COLOR_RGB2HSV), 
                    hsv_floor.low,
                    hsv_floor.high,
                    invert=True
                )
                cv2.imshow('Original', img)
                cv2.imshow('Without floor', floor_out_cropped)
                cv2.imshow('', getShapes(img, floor_out_cropped))
                cv2.waitKey(1)
                
            except:
                pass
            
            if self.stop.is_set():
                break
