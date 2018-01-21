import math

ZERO = -1e-9

class Point(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Vector(object):

    def __init__(self, A ,B):
        self.start_point = A
        self.end_point = B
        self.x = B.x - A.x
        self.y = B.y - A.y

