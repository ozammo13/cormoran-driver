import time

import cv2 as cv
import math as math
import numpy as np

from cormoran import Cormoran2WD

robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 36)  # initialises Robot, dont change this

# robot.connect_to_hardware() # connects to the robot. if the is removed then the code will run as a simulation
# robot.start()  # starts the robot, remove this line if you remove the connect hardware line


def drive(rotation):
    # robot.inputs =[wheel(radians),drive(meters a second)]
    robot.inputs = [rotation, 0.1]
    #feedback = robot.run_once()
    # print(feedback)
    time.sleep(1/50)


# threshold
hMin = 50
sMin = 100
vMin = 0
hMax = 61
sMax = 255
vMax = 255

dimensions = (640, 480)
whitecountL = 0
whitecountR = 0
thresh = 300

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

    # - Jared Stiles -
    # One time initialisation of the HSV values.
    if not completedCalibration:
        # make view a binary black/white first.
        calibration = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        result = cv.threshold(calibration, 127, 255, cv.THRESH_BINARY)[1]

        # Count the black and white pixels to make a descision.
        whiteCount = 0
        blackCount = 0
        for x in range (dimensions[0]):
            for y in range (dimensions[1]):
                if (result[y,x] == 255):
                    whiteCount += 1
                else:
                    blackCount += 1

        # Modify the HSV values based on the amount of black/white pixels.
        whiteDecimal = whiteCount / (dimensions[0]*dimensions[1])
        blackDecimal = blackCount / (dimensions[0]*dimensions[1])
        hMin = (whiteDecimal*10)+30
        sMin = math.degrees(math.atan(blackDecimal))+70
        vMin = 0        #fine
        hMax = 62       #todo
        sMax = 126      #fine
        vMax = 128      #fine

        completedCalibration = True
        # - Jared Stiles -
    
    stream = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    stream = cv.flip(stream, 0)
    stream = cv.blur(stream, [30, 30])
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
                whitecountL = whitecountL + 1

    for x in range(h):
        for i in range(w):

            if (right[x][i] == 255):  # if white
                whitecountR = whitecountR + 1
    # ----------------------------------------------------------------------------
    if whitecountL > whitecountR + thresh:
        print("driving right")
        drive(0.1)

    elif whitecountR > whitecountL + thresh:
        print("driving left")
        drive(-0.1)

    else:
        print("center")
        drive(0.0)

    whitecountL = 0
    whitecountR = 0
    cv.imshow('frame', stream)
    if cv.waitKey(1) == ord('q'):
        break
cameraOuput.release()
cv.destroyAllWindows()
