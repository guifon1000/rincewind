from geometry.Point import Point
import sys
import numpy as np
from geo_functions import cross, is_on_plane
from geometry.Vector import Vector
from geometry.Frame import Frame
class Plane(list):
    """
    defines a plane 
    """
    def __init__(self, *largs):
        self.reflection_mat = None
        if len(*largs) == 4:
            super(Plane, self).__init__(*largs)
        elif (len(*largs) == 3) and ([type(largs[0][i]) for i in range(3)] == [Point,Point,Point]):
            A = largs[0][0]
            B = largs[0][1]
            C = largs[0][2]
            v0 = Vector([B[i] - A[i] for i in range(3)])
            v1 = Vector([C[i] - A[i] for i in range(3)])
            v = cross(v0,v1)
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])
        elif (len(*largs) == 2) and ([type(largs[0][i]) for i in range(2)] == [Point,Vector]):
            A = largs[0][0]
            v = largs[0][1]
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])

    @property
    def normal_vector(self):
        return Vector([self[i] for i in range(3)]).unit()

    def reflection_matrix(self):
        mat = np.zeros((4,4), dtype = np.float)
        a = self[0]
        b = self[1]
        c = self[2]
        d = self[3]
        mat[0][0] = 1. - 2.*a**2.
        mat[0][1] =  - 2.*a*b
        mat[0][2] =  - 2.*a*c
        mat[0][3] =  - 2.*a*d
        mat[1][0] = -2.*a*b
        mat[1][1] = 1. - 2.*b**2.
        mat[1][2] = -2.*b*c
        mat[1][3] = -2.*b*d
        mat[2][0] = -2. * a* c
        mat[2][1] = -2. * b*c
        mat[2][2] = 1. - 2. * c**2.
        mat[2][3] = -2. * c * d
        mat[3][0] = 0.
        mat[3][1] = 0.
        mat[3][2] = 0.
        mat[3][3] = 1.
        self.reflection_mat = mat

    def reflect_point(self, pt):
        vpt = [pt[i] for i in range(3)]+[1.]
        self.reflection_matrix()
        return Point([float(np.dot(self.reflection_mat, vpt)[i]) for i in range(3)])

    def reflect_vector(self, vec):
        vec4 = [vec[i] for i in range(3)]+[0.]
        self.reflection_matrix()
        return  Vector([float(np.dot(self.reflection_mat, vec4)[i]) for i in range(3)])

    def reflect_frame(self, frame):
        self.reflection_matrix()
        return Frame([self.reflect_point(frame[0]), 
                      self.reflect_vector(frame[1]).unit(), 
                      self.reflect_vector(frame[2]).unit(), 
                      self.reflect_vector(frame[3]).unit()])
