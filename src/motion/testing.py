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
gray_frame = np.zeros((height, width), dtype=np.uint8);

lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])

print "width: ", width
print "height: ", height

l,h = 0,0
thresh = None

def command(msg):
    serialRead.write(msg.encode('ascii'))

def find_objects():
    # dilate the thresholded image to fill in holes, then find contours
    # on thresholded image
    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)

    # loop over the contours
    objects = []
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < args["min_area"]:
            continue

        # compute the bounding box for the contour, draw it on the frame,
        # and update the text
        objects += cv2.boundingRect(c)

return objects

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    now = time.time()
    retval, frame = cam.read() # Read frame

    if not retval:
        break;

	# resize the frame, convert it to grayscale, and blur it
	frame = imutils.resize(frame, width=500)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (21, 21), 0)

	# if the first frame is None, initialize it
	if firstFrame is None:
		firstFrame = gray
		continue

	# compute the absolute difference between the current frame and
	# first frame
	frameDelta = cv2.absdiff(firstFrame, gray)
	thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
	thresh = cv2.dilate(thresh, None, iterations=2)

    objects = find_objects()
    for obj in objects:
        draw_object(obj)

    # Show frames
    cv2.imshow(WIN_RF, frame);

	key = cv2.waitKey(1) & 0xFF

	# if the `q` key is pressed, break from the lop
	if key == ord("q"):
		break

command("s")

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
