import cv2
import numpy as np

class hsv:
    def __init__(self, low, high) -> None:
        self.low = low
        self.high = high

hsvFloor = hsv((0,0,0), (179, 200, 255))

def maskHsv(original, hsv, hsvLow, hsvHigh, invert = False):
    mask = cv2.inRange(hsv, hsvLow, hsvHigh)
    if invert:
        cropped = original - cv2.bitwise_and(original, original, mask=mask)
    else:
        cropped = cv2.bitwise_and(original, original, mask=mask)
    return mask, cropped

def getGray(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def getBlur(img):
    return cv2.GaussianBlur(img,(5,5),0)
    
def getThreshold(img):
    _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
    return thresh

def getAdaptativeThreshold(img):
    # return cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
    return img

def getContours(original, img, draw = True):
    contours = cv2.findContours(img, 1, 2)
    cnts = contours[0] if len(contours) == 2 else contours[1]

    if draw:
        for c in cnts:
            # compute the center of the contour
            cv2.drawContours(original, [c], -1, (255, 100, 255), 3)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                cv2.circle(original, (cX, cY), 1, (255, 255, 255), 1)
            
    return cnts, original

def getShapes(original, img):
    cnts, original = getContours(original, img)

    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        ratio = float(w)/h
        if ratio == 1:
            # ROI = original[y:y+h, x:x+w]
            cv2.putText(original, 'Square', (x-15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        else:
            cv2.putText(original, 'Shape', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
    return original

def getEdges(img):
    return cv2.Canny(img, 80, 150)

def getFeatures(original, img, feature_params):
    corners = cv2.goodFeaturesToTrack(getGray(img), **feature_params)
    if corners is not None:
        for x, y in np.float32(corners).reshape(-1, 2):
            cv2.circle(original, (int(x),int(y)), 10, (0, 255 , 0), 1)
    return original