import cv2

def getShapes(original, img):
    _original = original.copy()
    
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)
    contours = cv2.findContours(thresh, 1, 2)
    cnts = contours[0] if len(contours) == 2 else contours[1]

    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        ratio = float(w)/h
        if ratio >= 0.9 and ratio <= 1.5:
            ROI = _original[y:y+h, x:x+w]
            cv2.putText(_original, 'Square', c[0][0], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.rectangle(_original, (x, y), (x + w, y + h), (255,255,255), 2)
        elif ratio < 0.9:
            cv2.putText(_original, 'Rectangle', c[0][0], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.rectangle(_original, (x, y), (x + w, y + h), (255,255,255), 2)
        else:
            cv2.putText(_original, 'Shape', c[0][0], cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
    return _original
