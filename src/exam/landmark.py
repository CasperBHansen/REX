import numpy as np
import random_numbers as rn


thresh = 20

class Landmark(object):
    """Data structure for storing landmark information """
    
    def __init__(self, identifier, color, orientation):
        self.identifier = identifier
        self.color = color
        self.orientation = orientation

        # we don't know it's position until we see it
        self.x = None
        self.y = None

    def getIdentifier(self):
        return self.identifier

    def getColor(self):
        return self.color

    def getOrientation(self):
        return orientation

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setPosition(self, x, y):
        self.setX(x)
        self.setY(y)

    def match(self, orientation, color):
        r = np.max(self.color[0], color[0]) - np.min(self.color[0], color[0])
        g = np.max(self.color[1], color[1]) - np.min(self.color[1], color[1])
        b = np.max(self.color[2], color[2]) - np.min(self.color[2], color[2])

        return self.orientation == orientation and r < thresh and g < thresh and b < thresh
