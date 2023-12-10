from VisionProcessing.filters import getContours
from VisionProcessing.guiFeatures import white, writeText, drawingCircle, drawingContours

import cv2
import numpy as np

def drawEachContourAndCenter(img: np.ndarray, filtered: np.ndarray):
    img = img.copy()
    cnts = getContours(filtered)
    for c in cnts:
        # compute the center of the contour
        drawingContours(img, [c])
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            drawingCircle(img, (cX, cY), 1, white, 1)
    return img

# Shapes detection based on ratio
def matchShapes(img: np.ndarray, filtered: np.ndarray, draw: bool = True):
    cnts = getContours(filtered)
    if draw:
        img = drawEachContourAndCenter(img, filtered)
    else:
        img = img.copy()
    for i, c in enumerate(cnts):
        x, y, w, h = cv2.boundingRect(c)
        ratio = float(w)/h
        if ratio == 1:
            # ROI = original[y:y+h, x:x+w]
            writeText(img, f'Square_{i+1}', (x-15, y))
        elif ratio < 0.9 or ratio > 1.5:
            writeText(img, f'Rectangle_{i+1}', (x, y))
        else:
            writeText(img, f'Shape_{i+1}', (x, y))
    return img
