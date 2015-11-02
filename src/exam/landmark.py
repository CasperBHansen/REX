from particle import Particle

import numpy as np
import random_numbers as rn


class Landmark(Particle):
    """Data structure for storing landmark information """
    
    def __init__(self, identifier, location, color, orientation):
        self.identifier = identifier
        self.orientation = orientation
        self.universalx = location[0]
        self.universaly = location[1]
        self.color = color

        # we don't know it's position until we see it
        self.x = None
        self.y = None
        self.theta = None
        self.weight = None

    def getIdentifier(self):
        return self.identifier

    def getUX(self):
        return self.universalx

    def getUY(self):
        return self.universaly

    def getColor(self):
        return self.color

    def getOrientation(self):
        return self.orientation

    def setPosition(self, x, y):
        self.setX(x)
        self.setY(y)

    def match(self, orientation, colour):
        thresh = 0.25

        r = clamp_color(colour[0])
        g = clamp_color(colour[1])
        b = clamp_color(colour[2])

        rd = maximum(r, self.color[0]) - minimum(r, self.color[0])
        gd = maximum(g, self.color[1]) - minimum(g, self.color[1])
        bd = maximum(b, self.color[2]) - minimum(b, self.color[2])

        return self.orientation == orientation and rd < thresh and gd < thresh and bd < thresh

def minimum(a,b):
    if a < b:
        return a
    return b

def maximum(a,b):
    if a > b:
        return a
    return b

def clamp_color(color):
    if color < 0:
        return 0
    elif color > 255:
        return 255

    return color
