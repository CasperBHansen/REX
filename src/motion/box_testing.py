#!/usr/bin/python


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

"""
serialRead = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# wait for serial connection
print "Waiting for serial port connection ..."
while not serialRead.isOpen():
    sleep(1)
"""
port = '/dev/ttyACM0'
serialRead = serial.Serial(port,9600, timeout=1)

# Define some constants
lowThreshold=35;
ratio = 3;
kernel_size = 3;
pictak = 0
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
WIN_RF = "Motion Test";
#cv2.namedWindow(WIN_RF);
#cv2.moveWindow(WIN_RF, 100, 0);

# Preallocate memory
gray = np.zeros((height, width), dtype=np.uint8);
frameDelta = np.zeros((height, width), dtype=np.uint8);
thresh = np.zeros((height, width), dtype=np.uint8);

lower_blue = np.array([105,50,50])
upper_blue = np.array([140,255,255])
white = np.array([0,0,255])

print "width: ", width
print "height: ", height

l,h = 0,0
waittime = 0.0
nextcommand = "0"
degrees = 180.0 / np.pi

def command(msg):
    serialRead.write(msg.encode('ascii'))

def find_objects():
    if thresh == None:
        return []

    # dilate the thresholded image to fill in holes, then find contours
    # on thresholded image
    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)

    # loop over the contours
    objects = []
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < 20:
            continue

        # compute the bounding box for the contour, draw it on the frame,
        # and update the text
        objects.append(cv2.boundingRect(c))

    return objects

def filter_blue(mask):
    
    newmask = np.zeros((height, width), dtype=np.uint8)
    
    # dilate the thresholded image to fill in holes, then find contours
    # on thresholded image
    (cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)

    # loop over the contours
    biggest = 0 
    for i, c in enumerate(cnts):
        # if the contour is too small, ignore it
        if cv2.contourArea(c) > cv2.contourArea(cnts[biggest]):
            biggest = i

    cv2.drawContours(newmask,cnts,biggest, (255,255,255), 1)
    return newmask

def draw_object(obj):
	(x, y, w, h) = obj
	cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

reference = None
while cv2.waitKey(4) == -1: # Wait for a key pressed event
#while pictak == 0:
    now = time.time()
    retval, frame = cam.read() # Read frame

    # resize the frame, convert it to grayscale, and blur it
    #frame = imutils.resize(frame, width=500)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # if the first frame is None, initialize it
    if reference is None:
        reference = gray
        continue

    # compute the absolute difference between the current frame and
    # first frame
    frameDelta = cv2.absdiff(reference, gray)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    objects = find_objects()
    for obj in objects:
        draw_object(obj)

    blue_mask = np.zeros((height, width), dtype=np.uint8)
    (blue_cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    if time.time() > waittime:
        if nextcommand == "S":
            command('s/n')
            waittime = time.time()+1.0
            nextcommand = "0"
        elif blue_cnts:
            # loop over the contours
            biggest = 0 
            for i, c in enumerate(blue_cnts):
                # if the contour is too small, ignore it
                if cv2.contourArea(c) > cv2.contourArea(blue_cnts[biggest]):
                    biggest = i
            # find a simple square
            epsilon = 0.04*cv2.arcLength(blue_cnts[biggest],True)
            approx = cv2.approxPolyDP(blue_cnts[biggest],epsilon,True)
            print "number of points: ", len(approx)
            if len(approx) == 4:
                highest = 0
                lowest = 1000
                thinest = 1000
                thikest = 0
                side1H = 0
                side1L = 0
                side2H = 0
                side2L = 0
                approxset = [0,1,2,3]
                for i, a in enumerate(approx):
                    if a[0][1] < lowest:
                        lowest = a[0][1]
                        side1L = i
                    if a[0][1] > highest:
                        highest = a[0][1]
                        side1H = i
                for a in approx:
                    if a[0][0] < thinest:
                        thinest = a[0][0]
                    if a[0][0] > thikest:
                        thikest = a[0][0]
                
                itiration = 0
                while itiration < len(approxset):
                    if approxset[itiration] == side1H or approxset[itiration] == side1L:
                        approxset.remove(approxset[itiration])
                    else:
                        itiration += 1
                print approxset
                if approx[approxset[0]][0][1] > approx[approxset[1]][0][1]:
                    side2H = approxset[0]
                    side2L = approxset[1]
                else:
                    side2H = approxset[1]
                    side2L = approxset[0]
                if approx[side1H][0][0] > approx[side2H][0][0]:
                    rsH = side2H
                    rsL = side2L
                    lsH = side1H
                    lsL = side1L
                else:
                    rsH = side1H
                    rsL = side1L
                    lsH = side2H
                    lsl = side2L
                h = highest - lowest
                b = thikest - thinest
                print "approx: ", approx[rsH]
                print "b: ", b
                side1h = approx[side1H][0][1] - approx[side1L][0][1]
                side2h = approx[side2H][0][1] - approx[side2L][0][1]
                print "side1h: ", side1h
                print "side2h: ", side2h
                centerh = (float(side1h) + float(side2h)) / 2.0
                print "centerh: ", centerh
                centerd = ((658.59*29.7)/float(centerh))
                print "centerd: ", centerd
                distance = (658.59 * 29.7)/float(h)
                (centerx, centery),_ = cv2.minEnclosingCircle(approx)
                vf = (b * centerd)/(658.59 * 42.0)
                if vf < -1.0:
                    vf = -1.0
                elif vf > 1.0:
                    vf = 1.0
                v = np.arccos(vf)*degrees
                print "v foer arccos: ", vf
                theta = 180 - (90 + v)
                print "hieght: ", h
                print "distance: ", distance
                print "V = ", v
                print "theta: ", theta
                
                if nextcommand == "0":
                    if centerx < width/2.0 + 50.0 and centerx > width/2.0 - 50.0:
                        
                        command('s/n')
                        nextcommand = "F"
                        waittime = time.time() + 0.5
                    elif centerx > width/2.0 + 10.0:
                        command('p 270.0 0.9')
                        nextcommand = "S"
                        waittime = time.time()+0.1
                    else:
                        
                        command('p 90.0 0.9')
                        nextcommand = "S"
                        waittime = time.time()+0.1
                elif nextcommand == "S":
                    command('s/n')
                    nextcommand = "0"
                    waittime = time.time()+1.0

                elif nextcommand == "F":
                    nextcommand = "0"
                    '''if left < right:
                        print right
                        nextcommand = "0"
                    else:
                        print left
                        nextcommand = "0"'''
            cv2.drawContours(blue_mask,[approx], 0, (255,255,255), 1)
    
    # Show frames
    #cv2.imshow(WIN_RF, mask)
    #cv2.imshow(WIN_RF, blue_mask)
    #cv2.imshow("Thresh", thresh)
    #cv2.imshow("Frame Delta", frameDelta)
    #cv2.imwrite("testbillede.png", blue_mask)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key is pressed, break from the lop
    if key == ord("q"):
        break
    #pictak = 1
command("s/n")

# cleanup the camera and close any open windows
cam.release()
cv2.destroyAllWindows()
