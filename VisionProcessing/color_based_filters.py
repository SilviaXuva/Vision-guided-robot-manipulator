import cv2
import numpy as np
import os
from settings import Settings

if os.path.isfile(Settings.pre_processing_parameters_path):
    data = np.load(Settings.pre_processing_parameters_path)
else:
    data = {
        # Set range for floor color
        "floor_hsv_low": np.array([0,0,0]),
        "floor_hsv_high": np.array([179,200,255]),

        # Set range for red color
        "red_hsv_low":  np.array([61,0,0]),
        "red_hsv_high":  np.array([179,255,255]),

        # Set range for green color
        "green_hsv_low":  np.array([1, 0, 0]),
        "green_hsv_high":  np.array([116, 255, 255]),

        # Set range for blue color
        "blue_hsv_low":  np.array([0, 2, 2]),
        "blue_hsv_high":  np.array([58, 255, 255])
    }

# ranges = [
#     [data['red_hsv_low'], data['red_hsv_high']],
#     [data['green_hsv_low'], data['green_hsv_high']],
#     [data['blue_hsv_low'], data['blue_hsv_high']]
# ]
# ranges = {k: v for k, v in data.items() if k in [
#     'red_hsv_low', 'red_hsv_high', 'green_hsv_low', 'green_hsv_high', 'blue_hsv_low', 'blue_hsv_high'
# ]}
ranges = {
    "red": {
        "low": data['red_hsv_low'],
        "high": data['red_hsv_high']
    },
    "green": {
        "low": data['green_hsv_low'],
        "high": data['green_hsv_high']
    },
    "blue": {
        "low": data['blue_hsv_low'],
        "high": data['blue_hsv_high']
    }
}
# Color extraction based on HSV
def maskRanges(img, ranges = ranges):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    whole_mask = None
    colors = list()
    for key in ranges.keys():
        mask = cv2.inRange(hsv, ranges[key]['low'], ranges[key]['high'])
        if whole_mask is not None:
            whole_mask += mask
        else:
            whole_mask = mask
        if np.any(mask[:, :] == 255):
            colors.append(key)
    cropped = img.copy()
    cropped[np.where(whole_mask == 255)] = 255
    return cropped, colors

def removeBackground(img, hsv_low = data['floor_hsv_low'], hsv_high = data['floor_hsv_high']):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    cropped = img - cv2.bitwise_and(img, img, mask=mask)
    return cropped

def getGray(img, color = cv2.COLOR_RGB2GRAY):
    return cv2.cvtColor(img, color)