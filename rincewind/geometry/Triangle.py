import sys
import numpy as np
from .Vector import Vector
from .Point import Point
from .Plane import Plane
from .Line import Line
from ..geo_functions import cross, intersect_3_planes, closest_point_on_line, distance

lcar = 0.1

class Triangle(list):
    """    
               
                self[2]
                   ^
                 /  \
                /    \
           s3  /      \ s0
              /        \
             /     0    \
            v            \
     self[0] -----------> self[1]    (  0 = out normal )
                   s1
    """
    def __init__(self, *largs):
        super(Triangle, self).__init__(*largs)

    
    def __neg__(self):
        return Triangle((self[0], self[2], self[1]))

    @property
    def plane(self):
        return Plane(self)

    @property
    def cg(self):
        return Point(np.mean(self   ,axis=0))

    @property
    def normal(self):
        u = Vector((self[1][0]-self[0][0],\
    			self[1][1]-self[0][1],\
    			self[1][2]-self[0][2]))
        v = Vector((self[2][0]-self[1][0],\
    			self[2][1]-self[1][1],\
     			self[2][2]-self[1][2]))
        return cross(u,v)

    @property
    def circumcenter(self):
        plane_1 = Plane(self)
        mid_1 = Point([0.5 * (self[0][i] + self[1][i]) for i in range(3)])
        vnor_1 = Vector([mid_1[i] - self[0][i] for i in range(3)]).unit()
        plane_2 = Plane([mid_1, vnor_1])
        mid_2 = Point([0.5 * (self[0][i] + self[2][i]) for i in range(3)])
        vnor_2 = Vector([mid_2[i] - self[0][i] for i in range(3)]).unit()
        plane_3 = Plane([mid_2, vnor_2])
        try:
            return intersect_3_planes(plane_1, plane_2, plane_3)
        except:
            print('problem with circumcenter estimation for triangle '+str(self))
            print(plane_1)
            print(plane_2)
            print(plane_3)
            return None

    @property
    def orthocenter(self):
        line2 = Line([self[0], self[1]])
        prj2 = closest_point_on_line(self[2], line2)
        if distance(prj2, self[0])**2.< 1.e-10:
            nrm2 = Vector([self[1][i] - self[0][i] for i in range(3)]).unit()
        else:     
            nrm2 = Vector([prj2[i] - self[0][i] for i in range(3)]).unit()	
        plane2 = Plane([prj2, nrm2])
        line0 = Line([self[1], self[2]])
        prj0 = closest_point_on_line(self[0], line0)
        if distance(prj0, self[1])**2.< 1.e-10:
            nrm0 = Vector([self[2][i] - self[1][i] for i in range(3)]).unit()
        else:     
            nrm0 = Vector([prj0[i] - self[1][i] for i in range(3)]).unit()
        plane0 = Plane([prj0, nrm0])
        return intersect_3_planes(Plane(self), plane0, plane2)
    
    @property
    def incircle_center(self):
        return Point([0., 0., 0.])
		
    @property
    def area(self):
        line = Line([self[0], Vector([self[1][i]-self[0][i] for i in range(3)]).unit()])
        bas = distance(self[0], self[1])
        prj = closest_point_on_line(self[2], line)
        height = distance(self[2], prj)
        return 0.5*bas*height

    @property
    def distance_circumcenter_to_cg(self):
        return distance(self.cg, self.circumcenter)

    @property
    def distance_circumcenter_to_orthocenter(self):
        return distance(self.orthocenter, self.circumcenter)

    @property
    def mean_length(self):
        return (distance(self[0], self[1]) + distance(self[1], self[2]) + distance(self[2], self[0]))/3.

    @property
    def aspect_ratio(self):
        d0 = distance(self[0], self[1])
        d1 = distance(self[1], self[2])
        d2 = distance(self[2], self[0])
        return min([d0, d1, d2])/ max([d0, d1, d2])

    def frame(self):
        from rincewind.geometry.Frame import Frame
        pt = self[0]
        vecZ = self.normal.unit()
        vecX = Vector([self[1][i] - self[0][i] for i in range(3)]).unit()
        vecY = cross(vecZ, vecX).unit()
        return Frame([pt, vecZ, vecX,vecY])


    def add_to_ax(self,ax):
        x = [self[i][0] for i in range(3)]
        y = [self[i][1] for i in range(3)]
        z = [self[i][2] for i in range(3)]
        ax.plot_trisurf(x, y, z, linewidth=0.2, antialiased=False)
        #ax.plot(x, y, z)#, linewidth=0.2)
