import cv2

class hsv:
    def __init__(self, low, high) -> None:
        self.low = low
        self.high = high

hsv_floor = hsv((0,0,0), (179, 200, 255))

def maskHsv(original, hsv, hsvLow, hsvHigh, invert = False):
    mask = cv2.inRange(hsv, hsvLow, hsvHigh)
    if invert:
        cropped = original - cv2.bitwise_and(original, original, mask=mask)
    else:
        cropped = cv2.bitwise_and(original, original, mask=mask)
    return mask, cropped
