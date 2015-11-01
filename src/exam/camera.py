import cv2 # Import the OpenCV library
import numpy as np # We also need numpy
import time
import picamera

from picamera.array import PiRGBArray

# Black magic in order to handle differences in namespace names in OpenCV 2.4 and 3.0
from pkg_resources import parse_version
OPCV3 = parse_version(cv2.__version__) >= parse_version('3')

def capPropId(prop):
    """returns OpenCV VideoCapture property id given, e.g., "FPS
       This is needed because of differences in the Python interface in OpenCV 2.4 and 3.0
    """
    return getattr(cv2 if OPCV3 else cv2.cv, ("" if OPCV3 else "CV_") + "CAP_PROP_" + prop)

class Camera(object):
    def __init__(self, camidx, robottype='frindo'):
        """This class is responsible for doing the image processing. It detects known landmarks and 
    measures distances and orientations to these."""
        self.imageSize = (640, 480)
        self.intrinsic_matrix = np.asarray([ 7.1305391967046853e+02, 0., 3.1172820723774367e+02, 0.,
                   7.0564929862291285e+02, 2.5634470978315028e+02, 0., 0., 1. ], dtype = np.float64)
        self.intrinsic_matrix.shape = (3,3)
        self.distortion_coeffs = np.asarray([ 1.1911006165076067e-01, -1.0003366233413549e+00,
                   1.9287903277399834e-02, -2.3728201444308114e-03, -2.8137265581326476e-01 ], dtype = np.float64)

        self.cam = picamera.PiCamera(camidx)
        time.sleep(1) # wait for camera

        self.cam.resolution = (640, 480)
        self.rawCapture = PiRGBArray(self.cam, size=self.cam.resolution)    
        
        # Initializing the camera distortion maps
        #self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.intrinsic_matrix, self.distortion_coeffs, np.eye(3,3), np.eye(3,3), self.imageSize, cv2.CV_32FC1)
        # Not needed we use the cv2.undistort function instead
    
        # Initialize chessboard pattern parameters
        self.patternFound = False
        self.patternSize = (3,4)
        self.patternUnit = 50.0 # mm (size of one checker square)
        self.corners = []
    def get_capture(self):
        return self.cam
        
    def get_colour(self):
        """Gets the next frame and undistorts the image.
           Returns a tuple with the undistorted and distorted images"""
        
        distorted = []
        self.cam.capture(self.rawCapture, format="bgr")            
        frame = self.rawCapture
        #frame = self.cam.capture(format="bgr")
        distorted = frame.array
        print "taking a picture"
        # clear the stream in preparation for the next frame
        self.rawCapture.truncate(0)        
        
        #colour = cv2.remap(distorted, self.mapx, self.mapy, cv2.CV_INTER_LINEAR)
        #colour = cv2.remap(distorted, self.mapx, self.mapy, cv2.INTER_LINEAR)
        
        # Skip this part because it is slow
        #colour = cv2.undistort(distorted, self.intrinsic_matrix, self.distortion_coeffs)
        colour = distorted
        return colour, distorted
        
                
    def get_object(self, img):
        """Detect object and return object type, distance (in cm), angle (in radians) and colour probability table"""
        objectType = 'none'
        colourProb = np.ones((3,)) / 3.0
        distance = 0.0
        angle = 0.0
        self.patternFound = False
        
        #patternFound, corners = self.get_corners(img)
        self.get_corners(img)
        
        if (self.patternFound):
            
            # Determine if the object is horizontal or vertical
            delta_x = abs(self.corners[0,0,0] - self.corners[2,0,0])
            delta_y = abs(self.corners[0,0,1] - self.corners[2,0,1])
            horizontal = (delta_y > delta_x)
            if (horizontal):
                objectType = 'horizontal'
            else:
                objectType = 'vertical'
            
            # Compute distances and angles
            if (horizontal):
                height = ((abs (self.corners [0,0,1] - self.corners [2,0,1]) +
                          abs (self.corners [9,0,1] - self.corners [11,0,1])) / 2.0);
                          
                patternHeight = (self.patternSize[0]-1.0) * self.patternUnit
                
            else:
                height = (abs (self.corners [0,0,1] - self.corners [9,0,1]) +
                              abs (self.corners [2,0,1] - self.corners [11,0,1])) / 2.0;
                              
                patternHeight = (self.patternSize[1]-1.0) * self.patternUnit
            
            #distance = 1.0 / (0.0001 * height);
            distance = self.intrinsic_matrix[1,1] * patternHeight / (height * 10.0) 

            center = (self.corners [0,0,0] + self.corners [2,0,0] +
                                         self.corners [9,0,0] + self.corners [11,0,0]) / 4.0;
            
            #angle = 0.0018 * center - 0.6425;
            #angle *= -1.0;
            angle = -np.arctan2(center - self.intrinsic_matrix[0,2], self.intrinsic_matrix[0,0])
            
            
            #### Classify object by colour
            
            # Extract rectangle corners
            points = np.array(
                        [self.corners [0],
                        self.corners [2],
                        self.corners [9],
                        self.corners [11]]
                      )
                    
            points.shape = (4,2)
            points = np.int32(points)

            # Compute region of interest
            mask = np.zeros((self.imageSize[1], self.imageSize[0]), dtype =np.uint8) 
            cv2.fillConvexPoly (mask, points, (255, 255, 255))    
    
            # Compute mean colour inside region of interest
            mean_colour = cv2.mean(img, mask);
            
            red   = mean_colour[2];
            green = mean_colour[1];
            blue  = mean_colour[0];
            sum = red + green + blue;

            colourProb[0] = red / sum;
            colourProb[1] = green / sum;
            colourProb[2] = blue / sum;
            
            
        return objectType, distance, angle, colourProb
        
        
    def get_corners(self, img):
        """Detect corners - this is an auxillary method and should not be used directly"""
        
        # Convert to gray scale
        gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY );
        
        retval, self.corners = cv2.findChessboardCorners(gray, self.patternSize, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ADAPTIVE_THRESH)
        if (retval > 0):
            self.patternFound = True
            #cv2.cornerSubPix(gray, self.corners, (5,5), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3, 0.0))
        print "getting corners"    
        return self.patternFound, self.corners
        
        
    def draw_object(self, img):
        """Draw the object if found into img"""
        cv2.drawChessboardCorners(img, self.patternSize, self.corners, self.patternFound)

