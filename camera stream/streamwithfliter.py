import numpy as np
import cv2 as cv
# threshold
hMin = 36
sMin = 100
vMin = 0

hMax = 61
sMax = 255
vMax = 255
dim = (640, 480)
whitecountL = 0
whitecountR = 0
# Set minimum and maximum HSV values to display
lower = np.array([hMin, sMin, vMin])
upper = np.array([hMax, sMax, vMax])

cap = cv.VideoCapture(2)
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
                whitecountL = whitecountL + 1

    for x in range(h):
        for i in range(w):

            if (right[x][i] == 255):  # if white
                whitecountR = whitecountR + 1

    if whitecountL > whitecountR:
        print("left")
    else:
        print("right")
    whitecountL = 0
    whitecountR = 0
    cv.imshow('frame', stream)
    #cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
