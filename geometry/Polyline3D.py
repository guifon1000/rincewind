import numpy as np
from .Plane import Plane
from .Triangle import Triangle
from .Point import Point
from .Vector import Vector
from .Segment import Segment
from .Triangulation import Triangulation
from ..geo_functions import is_on_plane, distance, angle, cross, is_in_polyline
import random

class Polyline3D(list):    #always closed
    def __init__(self, *largs,**kwargs):
        super(Polyline3D,self).__init__(*largs)
        self.check_closed = False
        self.spline = None

    def spline_parameters(self):
        from ..geometry.ParametricCurve3D import Spline3D
        npt = len(self)
        deg = min(npt-1, 3)
        #print('---- SPLINE INTERPOLATION OF POLYLINE 3D ---------')
        #print('for '+str(npt)+' points, degree is '+str(deg))
        self.spline = Spline3D(self, degree = deg) .tck
        #print(self.spline[1])
        #print(np.transpose(self))
 
    def add_to_ax(self, ax):
        x = [p[0] for p in self]
        y = [p[1] for p in self]
        z = [p[2] for p in self]
        ax.plot(x,y,z,c='r') 

    def pop_to_geom(self, geom):
        pts = []
        lns = []
        for p in self:
            p = geom.add_point(p,0.1)
            pts.append(p)
        for i in range(len(pts)-1):
            l = geom.add_line(pts[i],pts[i+1])
            lns.append(l)
        return lns

    def is_planar(self):
        out = True
        if len(self)>=3:
            plane = Plane([self[0], self[1], self[2]])
            for p in self[3:]:
                if not is_on_plane(p, plane):
                    out = False
                    break
        return out

    


    def planar_polyline(self):
        if self.is_planar():
            plane = Plane([self[0], self[1], self[2]])
            normal = plane.normal_vector
            return plane
        else:
            print('polyline 3D is not planar !!!')

    
    def is_closed(self):
        return self[0] == self[-1]
 
    @property
    def gravity_center(self):
        if self.is_closed:
            return [np.mean([p[i] for p in self[:-1]]) for i in range(3)]
        else:
            return [np.mean([p[i] for p in self]) for i in range(3)]  
    
    def remove_duplicates(self, tol=1.e-8):
        cont = True
        while cont:
            for i,p in enumerate(self[:-1]):
                pp1 = self[i+1]
                if distance(p,pp1)<tol:
                    self.remove(pp1)
                    break
            else:
                cont = False

    def triangulate_planar(self,frame):
        from ..geometry.Polyline2D import Polyline2D
        """ 
        for now it should always be closed
        """
        pol2d = self.express_in_frame_plane(frame)
        if len(pol2d)==4:
            return [Triangle(pol2d.to_frame(frame)[0:3])]
            #return [Triangle(self[0], self[1], self[2])] 
        else:
            triangles = []
            while len(pol2d)>=5:
                for i,p in enumerate(pol2d):
                    if i==0:
                        prev = pol2d[-1]
                        nex = pol2d[i+1]
                    elif i==len(pol2d)-1:
                        prev = pol2d[i-1]
                        nex = pol2d[0]
                    else:
                        prev = pol2d[i-1]
                        nex = pol2d[i+1]
                    mean = [0.5*(prev[j]+nex[j]) for j in range(2)]
                    if is_in_polyline(mean, pol2d):
                        #triangles.append(Polyline2D([prev, p, nex, prev]))
                        new_pol = Triangle(Polyline2D([prev, p, nex, prev]).to_frame(frame)[0:3])
                        #triangles.append(Triangle([new_pol[j] for j in range(3)])) 
                        triangles.append(new_pol) 
                        pol2d.remove(p)
                        break
            if len(pol2d)==4:
                #triangles.append(pol2d)
                new_pol = pol2d.to_frame(frame)
                triangles.append(Triangle(new_pol[0:3])) 
                #triangles.append(Triangle([new_pol[j] for j in range(3)])) 
            return triangles
                            




    def express_in_frame_plane(self, frame):
        from ..geometry.Polyline2D import Polyline2D
        polyline2d = []
        for p in self:
            polyline2d.append(p.express_in_frame_plane(frame))
        return Polyline2D(polyline2d)

    def list_of_segments(self):
        segments = []
        for i in range(len(self)-1):
            pt1 = self[i]
            pt2 = self[i+1]
            segments.append(Segment([pt1, pt2]))
        return segments




    def integrate_lengthes(self):
        circ = 0.
        for i in range(len(self)-1):
            pt1 = self[i]
            pt2 = self[i+1]
            circ += distance(pt1, pt2)
        return circ

            

    def triangulate_first(self, normal): 
        if self.is_closed():
            dico = {'ref': self[:-1]}
        else:
            dico = {'ref': self[:]}
        triangles = []
        access_normal_vector = cross(Vector([dico['ref'][-1][i] - dico['ref'][0][i] for i in range(3)]).unit(), normal).unit()
        #access_normal_vector = normal

        dico = divide_eco(dico['ref'], dico, access_normal_vector)
        for k in dico:
            if 'ref' not in k:
                if dico[k].has_key('triangle'):
                    array_tri = dico[k]['triangle']
                    pt1 = Point(array_tri[0])
                    pt2 = Point(array_tri[1])
                    pt3 = Point(array_tri[2])
                    triangles.append(Triangle([pt1, pt2, pt3]))

        return triangles 
        
 
def divide(prevtri, polyline):
    if polyline.is_closed():
        pol = polyline[:-1]
    else:
        pol = polyline[:]
    if len(pol)==3:
        triangle = Triangle([pol[0], pol[1], pol[2]])
        return triangle.area, prevtri+[triangle], []
    elif len(pol)==2:
        return 0., prevtri, []
    polylines = []
    triangles = prevtri
    score = 1.e23
    point = None
    for pt in pol[1:-1]:
        triangle = Triangle([pol[0], pt, pol[-1]])
        pol1 = Polyline3D(pol[0:pol.index(pt)+1])
        pol2 = Polyline3D(pol[pol.index(pt):])
        #print(len(pol1), len(pol2))
        if len(pol1)<=2:
            score1 = 0.
            tri1 = []
            pols1 = []
        else:
            score1, tri1, pols1 = divide(prevtri+[triangle],pol1)
        if len(pol2)<=2:
            score2 = 0.
            tri2 = []
            pols2 = []
        else:
            score2, tri2, pols2 = divide(prevtri+[triangle],pol2)
        if score1 + score2 + triangle.area <= score:
            score = score1 + score2 + triangle.area
            triangles = prevtri+[triangle] + tri1+ tri2
            polylines = [pol1, pol2]
    return score,triangles, polylines           


def hash_list_of_integers(array):
    st = ''
    for i in array[:-1]:
        st+=str(i)+'_'
    st+=str(array[-1])
    return st

def set_of_hash(st):
    return set([int(s) for s in st.split('_')])
    

def divide_eco(polyline, dico, access_triangle_normal):
    if len(polyline)<=2:
        return dico
    idomain = [dico['ref'].index(pt) for pt in polyline]
    if not dico.has_key(hash_list_of_integers(idomain)):
        dico[hash_list_of_integers(idomain)] = {}
        pt_start = polyline[-1]
        pt_end = polyline[0]
        mincriterion = 1.e23
        domain1 = None
        domain2 = None
        kept_triangle = None
        for pt in polyline[1:-1]:
            idpt = polyline.index(pt)
            triangle = Triangle([polyline[-1], polyline[0], pt])
            criterion = 10.*abs(angle(access_triangle_normal, triangle.normal)) #+ triangle.area + triangle.distance_circumcenter_to_orthocenter
            if criterion < mincriterion:
                mincriterion = criterion
                kept_triangle = triangle
                domain1 = Polyline3D(polyline[idpt:])
                domain2 = Polyline3D(polyline[0:idpt+1])

        dico[hash_list_of_integers(idomain)]['triangle'] = kept_triangle
        divide_eco(domain1, dico, kept_triangle.normal)
        divide_eco(domain2, dico, kept_triangle.normal)
    return dico




