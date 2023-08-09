import cv2
import numpy as np
from spatialmath import SE3
from threading import Thread, Event
from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

class Vision(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.visionSensorHandle = sim.getObject('/Vision_sensor')
        self.defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
        sim.setInt32Param(sim.intparam_idle_fps, 0)
        client.setStepping(True)
        self.stop = Event()

        deg = np.pi/180        
        self.pose = SE3.Trans([+1.0000e+00, -1.8000e+00, +2.0000e+00])*SE3.Rx(-1.2952e+02*deg)*SE3.Ry(-2.4659e+01*deg)*SE3.Rz(-3.4386e+01*deg)
    
    def run(self):
        while True:
            img, resX, resY = sim.getVisionSensorCharImage(self.visionSensorHandle)
            try:
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

                # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
                # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
                # and color format is RGB triplets, whereas OpenCV uses BGR:
                # img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
                img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 1)

                cv2.imshow('LBR_iiwa Coppelia', img)
                cv2.waitKey(1)

                GetRedShapes(img)
                GetGreenShapes(img)
                GetBlueShapes(img)
                
            except:
                pass
            client.step()  # triggers next simulation step
            if self.stop.is_set():
                break

class VisionNonThreaded():
    def __init__(self):
        self.visionSensorHandle = sim.getObject('/Vision_sensor')
        self.defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
        sim.setInt32Param(sim.intparam_idle_fps, 0)
    
    def GetImg(self):
        img, resX, resY = sim.getVisionSensorCharImage(self.visionSensorHandle)
        try:
            img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

            # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
            # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
            # and color format is RGB triplets, whereas OpenCV uses BGR:
            img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

            processed = GetShape(img)
            cv2.imshow('', processed)
            cv2.waitKey(1)
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            ## Gen lower mask (0-70-0) and upper mask (0-255-255) of Red
            mask = cv2.inRange(hsv, (0-70-0), (0-255-255))
            ## Crop the Red regions
            croped = cv2.bitwise_and(img, img, mask=mask)         
            cv2.imshow('Only Red', croped)
            cv2.waitKey(0)
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            ## Gen lower mask (22-69-0) and upper mask (73-255-255) of Green
            mask = cv2.inRange(hsv, (22-69-0), (73-255-255))
            ## Crop the Green regions
            croped = cv2.bitwise_and(img, img, mask=mask)         
            cv2.imshow('Only Green', croped)
            cv2.waitKey(0)

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            ## Gen lower mask (79-64-0) and upper mask (179-255-255) of Blue
            mask = cv2.inRange(hsv, (79-64-0), (179,255,255))
            ## Crop the Blue regions
            croped = cv2.bitwise_and(img, img, mask=mask)         
            cv2.imshow('Only Blue', croped)
            cv2.waitKey(0)            
            
        except:
            pass

def GetCuboid(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    ret, thresh = cv2.threshold(gray,50,255,0)
    contours, hierarchy = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))

    for cnt in contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w)/h
            if ratio >= 0.9 and ratio <= 1.1:
                processed = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
                cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                processed = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)

    return processed

def GetShape(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret, th = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(th, 1, 2)
    # print("Number of contours detected:", len(contours))

    for cnt in contours:
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        if len(approx) == 1000:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w)/h
            if ratio >= 0.9 and ratio <= 1.1:
                processed = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
                cv2.putText(img, 'Square', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(img, 'Rectangle', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                processed = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)
        else:
            cv2.putText(img, 'Shape', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            processed = cv2.drawContours(img, [cnt], -1, (0,255,0), 3)

    return processed

def GetRedShapes(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ## Gen lower mask (0-70-0) and upper mask (0-255-255) of Red
    mask = cv2.inRange(hsv, (0,50,20), (5,255,255))
    ## Crop the Red regions
    croped = cv2.bitwise_and(img, img, mask=mask)         
    cv2.imshow('Only Red', croped)
    cv2.waitKey(1)
    
def GetGreenShapes(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ## Gen lower mask (22-69-0) and upper mask (73-255-255) of Green
    mask = cv2.inRange(hsv, (50,0,0), (86,255,255))
    ## Crop the Green regions
    croped = cv2.bitwise_and(img, img, mask=mask)         
    cv2.imshow('Only Green', croped)
    cv2.waitKey(1)
    
def GetBlueShapes(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ## Gen lower mask (79-64-0) and upper mask (179-255-255) of Blue
    mask = cv2.inRange(hsv, (100,150,0), (140,255,255))
    ## Crop the Blue regions
    croped = cv2.bitwise_and(img, img, mask=mask)         
    cv2.imshow('Only Blue', croped)
    cv2.waitKey(1)