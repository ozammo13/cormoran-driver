import numpy as np
import cv2 as cv
import math as math

# threshold as determined by hsv detect.py
hMin = 36
sMin = 100
vMin = 0
hMax = 61
sMax = 255
vMax = 255

dimensions = (640, 480)
whitecountL = 0
whitecountR = 0
completedCalibration = False

# Set minimum and maximum HSV values to display
lower = np.array([hMin, sMin, vMin])
upper = np.array([hMax, sMax, vMax])

cameraOuput = cv.VideoCapture(0)
if not cameraOuput.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cameraOuput.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    frame = cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

    if not completedCalibration:
        # make view a binary black/white
        calibration = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        result = cv.threshold(calibration, 127, 255, cv.THRESH_BINARY)[1]

        # count black and white pixels
        whiteCount = 0
        blackCount = 0
        for x in range (dimensions[0]):
            for y in range (dimensions[1]):
                if (result[y,x] == 255):
                    whiteCount += 1
                else:
                    blackCount += 1
        whiteDecimal = whiteCount / (dimensions[0]*dimensions[1])
        blackDecimal = blackCount / (dimensions[0]*dimensions[1])
        
        # modify values
        hMin = (whiteDecimal*10)+30
        sMin = math.degrees(math.atan(blackDecimal))+70
        vMin = 0        #fine
        hMax = 62       #todo
        sMax = 126      #fine
        vMax = 128      #fine

        completedCalibration = True
    
    stream = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    stream = cv.flip(stream, 0)
    stream = cv.blur(stream, [10, 10])
    stream = cv.inRange(stream, lower, upper)
    stream = stream[400:480, 100:540]  # NEW = 80,440

    
    # colour count---------------------------------------------------------------
    left = stream[0:80, 0:220]
    right = stream[0:80, 220:440]
    h = 80
    w = 220
    for x in range(h):
        for i in range(w):
            if (left[x][i] == 255):  # if white
                whitecountL += 1
            elif (right[x][i] == 255):
                whitecountR += 1     # if white

    if whitecountL > whitecountR:
        print("left")
    else:
        print("right")
        
    whitecountL = 0
    whitecountR = 0

    cv.imshow('stream', stream)
    cv.imshow('frame', frame)
    
    # If Q is pressed, terminate the application
    if cv.waitKey(1) == ord('q'):
        break
cameraOuput.release()
cv.destroyAllWindows()
