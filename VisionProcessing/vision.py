import cv2
import numpy as np

from Helpers.decorators import threaded

from VisionProcessing.color_based_filters import removeBackground, maskRanges, getGray
from VisionProcessing.filters import getBlur, getThreshold, getCanny, getCornersByGoodFeatures, getCornersByHarris, refineCorners
from VisionProcessing.segmentation import drawEachContourAndCenter, matchShapes
from VisionProcessing.gui_features import writeText
from settings import Settings

class ArUcoMarker():
    def __init__(self, corners, id) -> None:
        self.corners = corners
        self.id = id
        
class Vision():
    def __init__(self, files, aruco=False, aruco_dict=cv2.aruco.DICT_6X6_250, aruco_lenght=0.05):
        self.files = files
        self.count_save_img = 1
        self.index = 0

        self.aruco = aruco
        if aruco:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            self.aruco_lenght = aruco_lenght

        self.output = None
        self.draw = None
        self.getImage()

    @threaded
    def getImage(self):
        while True:
            if self.index == len(self.files):
                self.index = 0
            self.frame = cv2.imread(self.files[self.index])
            if self.output is None:
                self.output = self.frame.copy()
            self.draw = self.frame.copy()
            self.showImage()
            self.index += 1

    def showImage(self):
        for key in self.window.AllKeysDict.keys():
            if 'FRAME' in str(key) and str(key) != '-FRAME NUMBER-':
                self.window[key].update(data=cv2.imencode('.png', self.draw)[1].tobytes())
            elif 'PROCESSED' in str(key):
                self.window[key].update(data=cv2.imencode('.png', self.output)[1].tobytes())

    def preProcessing(self, event, values):
        floor_hsv_low = np.array([
            values['-H BACKGROUND SLIDER LOW-'],
            values['-S BACKGROUND SLIDER LOW-'],
            values['-V BACKGROUND SLIDER LOW-']
        ])
        floor_hsv_high = np.array([
            values['-H BACKGROUND SLIDER HIGH-'],
            values['-S BACKGROUND SLIDER HIGH-'],
            values['-V BACKGROUND SLIDER HIGH-']
        ])
        red_hsv_low = np.array([
            values['-H RED SLIDER LOW-'],
            values['-S RED SLIDER LOW-'],
            values['-V RED SLIDER LOW-']
        ])
        red_hsv_high = np.array([
            values['-H RED SLIDER HIGH-'],
            values['-S RED SLIDER HIGH-'],
            values['-V RED SLIDER HIGH-']
        ])
        green_hsv_low = np.array([
            values['-H GREEN SLIDER LOW-'],
            values['-S GREEN SLIDER LOW-'],
            values['-V GREEN SLIDER LOW-']
        ])
        green_hsv_high = np.array([
            values['-H GREEN SLIDER HIGH-'],
            values['-S GREEN SLIDER HIGH-'],
            values['-V GREEN SLIDER HIGH-']
        ])
        blue_hsv_low = np.array([
            values['-H BLUE SLIDER LOW-'],
            values['-S BLUE SLIDER LOW-'],
            values['-V BLUE SLIDER LOW-']
        ])
        blue_hsv_high = np.array([
            values['-H BLUE SLIDER HIGH-'],
            values['-S BLUE SLIDER HIGH-'],
            values['-V BLUE SLIDER HIGH-']
        ])

        if values['-NONE-']:
            self.output = self.frame

        if values['-MASK RANGES-']:
            self.output = maskRanges(self.output, [
                [red_hsv_low, red_hsv_high],
                [green_hsv_low, green_hsv_high],
                [blue_hsv_low, blue_hsv_high]
            ])
        elif values['-REMOVE BACKGROUND-']:
            self.output = removeBackground(self.frame, hsv_low = floor_hsv_low, hsv_high = floor_hsv_high)
        elif values['-RED MASK-']:
            self.output = maskRanges(self.output, [[red_hsv_low, red_hsv_high]])
        elif values['-GREEN MASK-']:
            self.output = maskRanges(self.output, [[green_hsv_low, green_hsv_high]])
        elif values['-BLUE MASK-']:
            self.output = maskRanges(self.output, [[blue_hsv_low, blue_hsv_high]])

        if values['-GRAY-']:
            gray = getGray(self.output)
            self.output = gray

        if values['-BLUR-']:
            blur = getBlur(self.output, 
                ksize = (
                    int(values['-BLUR SLIDER KSIZE-']), 
                    int(values['-BLUR SLIDER KSIZE-'])
                ), 
                sigmaX = values['-BLUR SLIDER SIGMAX-']
            )
            self.output = blur

        if values['-CANNY-']:
            canny = getCanny(self.output, 
                values['-CANNY SLIDER A-'], 
                values['-CANNY SLIDER B-']
            )
            self.output = canny

        if values['-THRESHOLD-']:
            if values['-THRESH SLIDER TYPE-'] == 'THRESH_BINARY':
                type = cv2.THRESH_BINARY

            thresh = getThreshold(self.output, 
                thresh = values['-THRESH SLIDER THRESH-'], 
                maxval = values['-THRESH SLIDER MAXVAL-'], 
                type = type
            )
            self.output = thresh

        if values['-CORNERS_GF-']:
            corners, img_corners_gf = getCornersByGoodFeatures(self.frame, self.output, True,
                int(values['-CORNERS_GF SLIDER MAXCORNERS-']),
                values['-CORNERS_GF SLIDER QUALITYLEVEL-'], 
                values['-CORNERS_GF SLIDER MINDISTANCE-']
            )
            self.output = img_corners_gf

        if values['-CORNERS_H-']:
            corners, img_corners_h = getCornersByHarris(self.frame, self.output, True,
                int(values['-CORNERS_H SLIDER BLOCKSIZE-']),
                int(values['-CORNERS_H SLIDER KSIZE-']), 
                values['-CORNERS_H SLIDER K-']
            )
            self.output = img_corners_h

        if values['-DRAW CONTOURS-']:
            self.output = drawEachContourAndCenter(self.frame, self.output)

        if values['-MATCH SHAPES-']:
            self.output = matchShapes(self.frame, self.output)

        if event == 'Save':
            np.savez(fr'{Settings.execution_path}\pre_processing_parameters.npz', **{
                "floor_hsv_low": np.array([
                    values['-H BACKGROUND SLIDER LOW-'], 
                    values['-H BACKGROUND SLIDER LOW-'], 
                    values['-H BACKGROUND SLIDER LOW-']
                ]),
                "floor_hsv_high": np.array([
                    values['-H BACKGROUND SLIDER HIGH-'], 
                    values['-H BACKGROUND SLIDER HIGH-'], 
                    values['-H BACKGROUND SLIDER HIGH-']
                ]),

                "red_hsv_low": np.array([
                    values['-H RED SLIDER LOW-'], 
                    values['-H RED SLIDER LOW-'], 
                    values['-H RED SLIDER LOW-']
                ]),
                "red_hsv_high": np.array([
                    values['-H RED SLIDER HIGH-'], 
                    values['-H RED SLIDER HIGH-'], 
                    values['-H RED SLIDER HIGH-']
                ]),

                "green_hsv_low": np.array([
                    values['-H GREEN SLIDER LOW-'], 
                    values['-H GREEN SLIDER LOW-'], 
                    values['-H GREEN SLIDER LOW-']
                ]),
                "green_hsv_high": np.array([
                    values['-H GREEN SLIDER HIGH-'], 
                    values['-H GREEN SLIDER HIGH-'], 
                    values['-H GREEN SLIDER HIGH-']
                ]),

                "blue_hsv_low": np.array([
                    values['-H BLUE SLIDER LOW-'], 
                    values['-H BLUE SLIDER LOW-'], 
                    values['-H BLUE SLIDER LOW-']
                ]),
                "blue_hsv_high": np.array([
                    values['-H BLUE SLIDER HIGH-'], 
                    values['-H BLUE SLIDER HIGH-'], 
                    values['-H BLUE SLIDER HIGH-']
                ]),

                "blur_ksize": (values['-BLUR SLIDER KSIZE-'], values['-BLUR SLIDER KSIZE-']),
                "blur_sigmax": values['-BLUR SLIDER SIGMAX-'],
                "canny_a": values['-CANNY SLIDER A-'],
                "canny_b": values['-CANNY SLIDER B-'],
                "thresh_thresh": values['-THRESH SLIDER THRESH-'],
                "thresh_maxval": values['-THRESH SLIDER MAXVAL-'],
                "corners_gf_maxcorners": values['-CORNERS_GF SLIDER MAXCORNERS-'],
                "corners_gf_qualitylevel": values['-CORNERS_GF SLIDER QUALITYLEVEL-'],
                "corners_gf_mindistance": values['-CORNERS_GF SLIDER MINDISTANCE-'],
                "corners_h_blocksize": values['-CORNERS_H SLIDER BLOCKSIZE-'],
                "corners_h_ksize": values['-CORNERS_H SLIDER KSIZE-'],
                "corners_h_k": values['-CORNERS_H SLIDER K-'],
            })
            cv2.imwrite(fr'{Settings.execution_path}\frame{self.count_save_img:02}.png', self.output)
            Settings.log(f'frame{self.count_save_img:02}', values)
            self.count_save_img += 1
        
    def detectArUco(self):
        corners, ids, rejectedCandidates = self.detector.detectMarkers(self.output)
        self.aruco_markers = list()

        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(self.draw, corners)

        # If markers are detected
        if len(corners) > 0:
            for (marker_corners, marker_ID) in zip(corners, ids):
                self.aruco_markers.append(ArUcoMarker(marker_corners, marker_ID[0]))

                writeText(self.draw, marker_ID[0], (int(marker_corners[0][0][0]), int(marker_corners[0][0][1])))