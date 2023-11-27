import cv2
import numpy as np
import os
from settings import Settings
from VisionProcessing.gui_features import drawingCircle, pink

if os.path.isfile(Settings.pre_processing_parameters_path):
    data = np.load(Settings.pre_processing_parameters_path)
else:
    data = {
        "blur_ksize": (5,5),
        "blur_sigmax": 0,
        "canny_a": 10,
        "canny_b": 50,
        "thresh_thresh": 0,
        "thresh_maxval": 255,
        "corners_gf_maxcorners": 100,
        "corners_gf_qualitylevel": 0.01,
        "corners_gf_mindistance": 10,
        "corners_h_blocksize": 2,
        "corners_h_ksize": 3,
        "corners_h_k": 0.04,
    }

# ============== Blur ===================
def getBlur(img, ksize = data["blur_ksize"], sigmaX = data["blur_sigmax"]):
    blur = cv2.GaussianBlur(img, ksize, sigmaX)
    return blur

# ============== Canny ===================
def getCanny(img, threshold1 = data["canny_a"], threshold2 = data["canny_b"]):
    canny = cv2.Canny(img, threshold1, threshold2)
    return canny
    
# ============== Threshold ===================
def getThreshold(img, thresh = data["thresh_thresh"], maxval = data["thresh_maxval"], type = cv2.THRESH_BINARY):
    ret, threshold = cv2.threshold(img, thresh, maxval, type)
    return threshold

# ============== Contours ===================
def getContours(img, mode = 1, method = 2):
    contours = cv2.findContours(img, mode, method)
    cnts = contours[0] if len(contours) == 2 else contours[1]
    return cnts

# ============== Corners By Good Features ===================
def getCornersByGoodFeatures(img, filtered, draw = False, maxCorners = data["corners_gf_maxcorners"], qualityLevel = data["corners_gf_qualitylevel"], minDistance = data["corners_gf_mindistance"]):
    corners = cv2.goodFeaturesToTrack(filtered, maxCorners, qualityLevel, minDistance)
    if draw:
        img = img.copy()
        try:
            corners = np.intp(corners)
            for corner in corners:
                x,y = corner.ravel()
                drawingCircle(img, (x,y))
                drawingCircle(img, (x,y))
        except:
            print('Good features to track returned blank')
    return corners, img

# ============== Corners By Harris ===================
def getCornersByHarris(img, filtered, draw = False, blockSize = data["corners_h_blocksize"], ksize = data["corners_h_ksize"], k = data["corners_h_k"]):
    dst = cv2.cornerHarris(np.float32(filtered), blockSize, ksize, k)
    if draw:
        img[dst > 0.01 * dst.max()] = pink
    return dst, img

# ============== Refine Corners ===================
def refineCorners(img, corners, winSize = (11, 11), zeroZone = (-1, -1), criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)):
    corners = cv2.cornerSubPix(img, corners, winSize, zeroZone, criteria)
    return corners

# ============== Not working ===================
# def getAdaptativeThreshold(img):
#     return cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
# ==============================================