#!/usr/bin/python

# This script shows how to do simple processing on frames comming from a camera in OpenCV.
# Kim S. Pedersen, 2015

import time
from time import sleep

import serial

import cv2 # Import the OpenCV library
import numpy as np # We also need numpy

from pkg_resources import parse_version
OPCV3 = parse_version(cv2.__version__) >= parse_version('3')

def capPropId(prop):
    """returns OpenCV VideoCapture property id given, e.g., "FPS
       This is needed because of differences in the Python interface in OpenCV 2.4 and 3.0
    """
    return getattr(cv2 if OPCV3 else cv2.cv, ("" if OPCV3 else "CV_") + "CAP_PROP_" + prop)
    
    
serialRead = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# wait for serial connection
print "Waiting for serial port connection ..."
while not serialRead.isOpen():
    sleep(1)

# Define some constants
lowThreshold=35;
ratio = 3;
kernel_size = 3;


# Open a camera device for capturing
cam = cv2.VideoCapture(0);

if not cam.isOpened(): # Error
    print "Could not open camera";
    exit(-1);
    
# Get camera properties
#cam.set(capPropId("FRAME_WIDTH"), 640/4)
#cam.set(capPropId("FRAME_WIDTH"), 480/4)
width = int(cam.get(capPropId("FRAME_WIDTH"))); 
height = int(cam.get(capPropId("FRAME_HEIGHT")));
        
    
# Open a window
WIN_RF = "Eksempel 2";
cv2.namedWindow(WIN_RF);
cv2.moveWindow(WIN_RF, 100       , 0);


# Preallocate memory
#gray_frame = np.zeros((height, width), dtype=np.uint8);

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

print "width: ", width
print "height: ", height

l,h = 0,0

def command(msg):
    serialRead.write(msg.encode('ascii'))


last = 0.0

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    now = time.time()
    retval, frame = cam.read() # Read frame
    
    if not retval: # Error
        print " < < <  Game over!  > > > ";
        exit(-1);

    if l == 0 or h == 0:
        l = frame.shape[1]
        h = frame.shape[0]

        bandwidth = int(l*0.38)

        mask_left = np.zeros((h,l,1),np.uint8)
        mask_right = np.zeros((h,l,1),np.uint8)

        for i in xrange(h):
            for j in xrange(bandwidth):
                mask_left[i][j] = 1
                mask_right[i][l-j-1] = 1

    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # mask, blue filter
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    (val_left, _, _, _) = cv2.mean(mask, mask=mask_left)
    (val_right, _, _, _) = cv2.mean(mask, mask=mask_right)

    # print "left: ", val_left
    #print "right: ", val_right

    mean = (val_left + val_right) / 2

    if now - last > 0.5:
        if val_left < val_right:
            command("a 90.0")
            command("v 1.0")
        else:
            command("a -90.0")
            command("v 1.0")

        last = now
    
    # Show frames
    cv2.imshow(WIN_RF, res);

command("s")
