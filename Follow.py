import time

import cv2 as cv
import numpy as np
import sys
from cormoran import Cormoran2WD

robot = Cormoran2WD(['208737A03548', '307F347D3131'],
                    wheelbase=0.0254 * 24, track_width=0.0254 * 36)  # initialises Robot, dont change this

# connects to the robot. if the is removed then the code will run as a simulation
# robot.connect_to_hardware()
# robot.start()  # starts the robot, remove this line if you remove the connect hardware line


def drive(rotation):
    if whitecountL < 50 or whitecountR < 50:
        rotation = 0.2
        print("rotation increase")
    robot.inputs = [rotation, 0.1]
    feedback = robot.run_once()
    print(feedback)
    time.sleep(1/50)


# threshold
hMin = 21
sMin = 0
vMin = 0

hMax = 44
sMax = 255
vMax = 255
dim = (640, 480)
whitecountL = 0
whitecountR = 0
thresh = 300
# Set minimum and maximum HSV values to display

lower = np.array([hMin, sMin, vMin])
upper = np.array([hMax, sMax, vMax])

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:

    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)
    stream = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    flipped = cv.flip(stream, 0)
    stream = cv.blur(flipped, [30, 30])
    filtered = cv.inRange(stream, lower, upper)
    stream = filtered[0:80, 0:640]  # NEW = 80,440

    # colour count---------------------------------------------------------------
    left = stream[0:80, 0:320]
    right = stream[0:80, 320:640]
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

    # elif whitecountL < 100 and whitecountR < 100:
    #    sys.exit()

    else:
        print("center")
        drive(0.0)

    whitecountL = 0
    whitecountR = 0
    cv.imshow('frame', stream)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
