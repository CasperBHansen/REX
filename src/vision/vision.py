#!/usr/bin/python2

import sys
sys.path.append("/usr/lib/python2.7/site-packages")

import time
from time import sleep

import cv2 # Import the OpenCV library
import numpy as np # We also need numpy

# Open a window
WIN_RF = "Vision Test";
cv2.namedWindow(WIN_RF);
cv2.moveWindow(WIN_RF, 100, 0);

while cv2.waitKey(4) == -1:

    frame = cv2.imread("left.png")
    bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(bw, 100, 100, apertureSize=3)
    lines = cv2.HoughLines(edges, 4, np.pi/180, 200)

    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)

        x0 = a*rho
        y0 = b*rho

        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(frame, (x1,y1), (x2,y2), (0,255,0), 1)

    # Show frames
    cv2.imshow(WIN_RF, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
