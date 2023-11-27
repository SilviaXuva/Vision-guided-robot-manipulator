import numpy as np

from VisionProcessing.color_based_filters import removeBackground, maskRanges, getGray
from VisionProcessing.filters import getBlur, getThreshold, getCanny, getCornersByGoodFeatures, getCornersByHarris, refineCorners
from VisionProcessing.segmentation import drawEachContourAndCenter, matchShapes
from VisionProcessing.gui_features import writeText

def preProcessing(window, event, values, frame):
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
        output = frame

    if values['-MASK RANGES-']:
        output = maskRanges(output, [
            [red_hsv_low, red_hsv_high],
            [green_hsv_low, green_hsv_high],
            [blue_hsv_low, blue_hsv_high]
        ])
    elif values['-REMOVE BACKGROUND-']:
        output = removeBackground(frame, hsv_low = floor_hsv_low, hsv_high = floor_hsv_high)
    elif values['-RED MASK-']:
        output = maskRanges(output, [[red_hsv_low, red_hsv_high]])
    elif values['-GREEN MASK-']:
        output = maskRanges(output, [[green_hsv_low, green_hsv_high]])
    elif values['-BLUE MASK-']:
        output = maskRanges(output, [[blue_hsv_low, blue_hsv_high]])

    if values['-GRAY-']:
        gray = getGray(output)
        output = gray

    if values['-BLUR-']:
        blur = getBlur(output, 
            ksize = (
                int(values['-BLUR SLIDER KSIZE-']), 
                int(values['-BLUR SLIDER KSIZE-'])
            ), 
            sigmaX = values['-BLUR SLIDER SIGMAX-']
        )
        output = blur

    if values['-CANNY-']:
        canny = getCanny(output, 
            values['-CANNY SLIDER A-'], 
            values['-CANNY SLIDER B-']
        )
        output = canny

    if values['-THRESHOLD-']:
        if values['-THRESH SLIDER TYPE-'] == 'THRESH_BINARY':
            type = cv2.THRESH_BINARY

        thresh = getThreshold(output, 
            thresh = values['-THRESH SLIDER THRESH-'], 
            maxval = values['-THRESH SLIDER MAXVAL-'], 
            type = type
        )
        output = thresh

    if values['-CORNERS_GF-']:
        corners, img_corners_gf = getCornersByGoodFeatures(frame, output, True,
            int(values['-CORNERS_GF SLIDER MAXCORNERS-']),
            values['-CORNERS_GF SLIDER QUALITYLEVEL-'], 
            values['-CORNERS_GF SLIDER MINDISTANCE-']
        )
        output = img_corners_gf

    if values['-CORNERS_H-']:
        corners, img_corners_h = getCornersByHarris(frame, output, True,
            int(values['-CORNERS_H SLIDER BLOCKSIZE-']),
            int(values['-CORNERS_H SLIDER KSIZE-']), 
            values['-CORNERS_H SLIDER K-']
        )
        output = img_corners_h

    if values['-DRAW CONTOURS-']:
        output = drawEachContourAndCenter(frame, output)

    if values['-MATCH SHAPES-']:
        output = matchShapes(frame, output)

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
        cv2.imwrite(fr'{Settings.execution_path}\frame{count_save_img:02}.png', output)
        Settings.log(f'frame{count_save_img:02}', values)
        count_save_img += 1