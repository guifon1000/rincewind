from scipy import interpolate
from geometry.Point import Point
from geometry.Vector import Vector
from geometry.Polyline3D import Polyline3D
from geometry.Plane import Plane
from geometry.Frame import Frame
from geo_functions import closest_point_on_plane, cross
import numpy as np

class ParametricCurve3D(object):
    def __init__(self, funcX, funcY, funcZ):
        self.funcX = funcX
        self.funcY = funcY
        self.funcZ = funcZ

    def parameter_point(self, s):
        return Point([self.funcX(s), self.funcY(s), self.funcZ(s)])

    def tangent_vector(self,s):
        #if s==1. :s-=0.001   #crade
        t = self.parameter_point(s)
        orig = t
        tph = self.parameter_point(s+0.001)
        return Vector( [float(tph[0]-t[0]), float(tph[1]-t[1]), float(tph[2]-t[2]) ]).unit()

    def discretize(self, n=20):
        pol3d = []
        for i in range(n):
            s = float(i)/(float(n))
            pol3d.append(self.parameter_point(s))
        return Polyline3D(pol3d)

    def frame_array_with_rmf(self, frame_zero, n=20):
        from geo_functions import rotation_minimized_frame
        p0 = self.parameter_point(0.)
        tg = self.tangent_vector(0.)
        pm1 = Point([p0[i] - tg[i] for i in range(3)])
        frame_m1 = Frame([pm1, frame_zero[1], frame_zero[2], frame_zero[3]])
        frames = []
        frames.append( Frame([p0, frame_zero[1], frame_zero[2], frame_zero[3]]))
        for i in range(n):
            s = float(i+1)/float(n)
            p = self.parameter_point(s)
            tg = self.tangent_vector(s)
            frame = rotation_minimized_frame(frame_m1, p, tg)
            frames.append(frame)
            frame_m1 = frame
        return frames







    def frame_array(self, mode = Vector([0.,0.,1.]), n=20, mirror = False):
        a_frame = []
        for i in range(n):
            s = float(i)/(float(n))
            a_frame.append(self.parameter_frame(s,mode = mode))
        return a_frame

    def symmetrize(self, smooth = False, plane_normal = Vector([0.,1.,0.])):
        # we will always take the first point of the control polyline as origin of the symmetry plane
        new_pol3d = [self.control_polyline[0]]
        plane = Plane([self.control_polyline[0], plane_normal.unit()])
        for p in self.control_polyline[1:]:
            cp = closest_point_on_plane(p, plane)
            print('closest of '+str(p) + ' is '+str(cp))
            vec = Vector([cp[i] - p[i] for i in range(3)])
            new_pol3d.append(Point([cp[i] + vec[i] for i in range(3)]))
        return Spline3D(new_pol3d)

    def parameter_frame(self, s, mode = Vector([0.,0.,1.])):
        basis = []
        #if s==1. :s-=0.001   #crade
        t = self.parameter_point(s)
        orig = t
        if (type(mode)  ==  Vector):
            vec = mode.unit()
        elif mode == 'Xnat':
            vec = (1.,0.,0.) 
        elif mode == 'Ynat':
            vec = (0.,1.,0.) 
        elif mode == 'Znat':
            vec = (0.,0.,1.)
        
        tph = self.parameter_point(s+0.001)
        xp  = Vector( [float(tph[0]-t[0]), float(tph[1]-t[1]), float(tph[2]-t[2]) ])
        T = xp.unit()
        basis.append(T)
        B = cross(vec, T)
        basis.append(B)
        N = cross(T, B)
        basis.append(N)

        matrix = [None,None,None]
        for i in range(3) : matrix[i] = [float(basis[i][j]) for j in range(3)]
        print(vec)
        print(matrix)
        print('-----------')
        return Frame((orig, matrix[0], matrix[1], matrix[2]))

    def add_to_ax(self, ax, n= 100):
        parameters = [float(i)/float(n-1) for i in range(n)]
        x = [self.parameter_point(s)[0] for s in parameters]
        y = [self.parameter_point(s)[1] for s in parameters]
        z = [self.parameter_point(s)[2] for s in parameters]
        ax.plot(x,y,z)

class Spline3D(object):
    def __init__(self, a_Point, degree=3):
        x = [p[0] for p in a_Point]
        y = [p[1] for p in a_Point]
        z = [p[2] for p in a_Point]
        self.control_polyline = Polyline3D(a_Point)
        self.tck, self.u = interpolate.splprep([x,y,z], k=degree)

    def parameter_point(self, s):
        t =  interpolate.splev(s , self.tck, der = 0)
        return Point([float(t[i]) for i in range(3)])

    def tangent_vector(self,s):
        t = self.parameter_point(s)
        orig = t
        tph = self.parameter_point(s+0.001)
        return Vector( [float(tph[0]-t[0]), float(tph[1]-t[1]), float(tph[2]-t[2]) ]).unit()

    def discretize(self, n=20):
        pol3d = []
        for i in range(n):
            s = float(i)/(float(n))
            pol3d.append(self.parameter_point(s))
        return Polyline3D(pol3d)

    def frame_array(self, mode = Vector([0.,0.,1.]), n=20, mirror = False):
        a_frame = []
        for i in range(n):
            s = float(i)/(float(n))
            a_frame.append(self.parameter_frame(s,mode = mode))
        return a_frame

    def symmetrize(self, smooth = False, plane_normal = Vector([0.,1.,0.])):
        # we will always take the first point of the control polyline as origin of the symmetry plane
        new_pol3d = [self.control_polyline[0]]
        plane = Plane([self.control_polyline[0], plane_normal.unit()])
        for p in self.control_polyline[1:]:
            cp = closest_point_on_plane(p, plane)
            print('closest of '+str(p) + ' is '+str(cp))
            vec = Vector([cp[i] - p[i] for i in range(3)])
            new_pol3d.append(Point([cp[i] + vec[i] for i in range(3)]))
        return Spline3D(new_pol3d)

    def parameter_frame(self, s, mode = 'frenet'):
        basis = []
        t =  interpolate.splev(s , self.tck, der = 0)
        orig = Point([float(t[i]) for i in range(3)])
        if mode == 'frenet':
            t =  interpolate.splev(s , self.tck, der = 1)
            xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
            t =  interpolate.splev(s , tck, der = 2)
            xpp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
            T = xp.unit()
            basis.append(T.unit())
            B = cross(xp, xpp)
            basis.append(B.unit())
            N = cross(B, T)
            basis.append(N.unit())
        else:
            if (type(mode)  ==  Vector):
                vec = mode.unit()
            elif mode == 'Xnat':
                vec = (1.,0.,0.) 
            elif mode == 'Ynat':
                vec = (0.,1.,0.) 
            elif mode == 'Znat':
                vec = (0.,0.,1.) 
            t =  interpolate.splev(s , self.tck, der = 1)
            xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
            T = xp.unit()
            basis.append(T)
            B = cross(vec, T)
            basis.append(B)
            N = cross(T, B)
            basis.append(N)
  
            matrix = [None,None,None]
            for i in range(3) : matrix[i] = [float(basis[i][j]) for j in range(3)]
            return Frame((orig, matrix))
