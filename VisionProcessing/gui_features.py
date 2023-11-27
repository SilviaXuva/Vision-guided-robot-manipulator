import cv2

blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
white = (255, 255, 255)
yellow = (255, 255, 0)
pink = (255, 100, 255)
black = (0, 0, 0)

def writeText(img, text, origin, fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, color = pink, thickness = 1):
    cv2.putText(img, str(text), origin, fontFace, fontScale, color, thickness)

# Drawing Circle
def drawingCircle(img, center, radius = 3, color = pink, thickness = -1):
    cv2.circle(img, center, radius, color, thickness)

# Drawing Rectangle
def drawingRectangle(img, pt1, pt2, color = pink, thickness = 2):
    cv2.rectangle(img, pt1, pt2, color, thickness)

# Drawing Countours
def drawingContours(img, contours, contourIdx = -1, color = pink, thickness = 3):
    cv2.drawContours(img, contours, contourIdx, color, thickness)