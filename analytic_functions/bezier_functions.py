import numpy as np
from scipy.special import binom as spbinom

def binom(n,k):
    if k<=n:
        return factorial(n)/(factorial(k)*factorial(n-k))

def pascal_triangle(level):
    for i in range(level):
        s=''
        for j in range(i+1):
            s+=str(binom(i,j))+' '
        print(s)

def factorial(n):
    if n==1 or n==0:
        return 1
    else:
        return n*factorial(n-1)


def de_casteljau(control_points, parameter):
    def elementary_step(array):
        out_array=[]
        for i,p in enumerate(array[:-1]):
            pt = [p[j] * (1.-parameter) + array[i+1][j] * parameter for j in range(2)]
            out_array.append(pt)
        return out_array
    ar = control_points
    for i in range(len(control_points)-1):
        ar = elementary_step(ar)
    return  ar[0]

def linear_function(value1, value2):
    f = lambda x: value1 + (value2 - value1) * x
    return f



def piecewise_bezier_polyline(start_value = 0., end_value = 0., *largs):
    #import matplotlib.pyplot as plt
    from ..geometry.Point import Point
    from ..geometry.Vector import Vector
    from ..geometry.Line import Line
    from ..geo_functions import cross, dot, angle, intersect_2_lines
    prev = [0., start_value]
    last = [1.0, end_value]
    bezier_parts = [[prev]]
    #plt.clf()
    #plt.axis('equal')
    #plt.scatter([prev[0], last[0]],[prev[1], last[1]], c= 'k')
    if largs:
        for i,cpl in enumerate(largs[0]):
            if i == len(largs[0])-1:
                nex = last
            else:
                nex = largs[0][i+1]
            pt = [cpl[0], cpl[1]]
            tmppt = Point([pt[0], pt[1], 0.])
            #plt.scatter(pt[0], pt[1], c='k')
            rad = cpl[2]
            vec1 = Vector([pt[0] - prev[0], pt[1] - prev[1], 0.])
            vec2 = Vector([nex[0] - pt[0], nex[1] - pt[1], 0.])
            Z = Vector([0., 0., 1.])
            ang = angle(vec1, vec2, plane_normal=Z)
            if ang>0.:sgn = 1.
            elif ang<0.:sgn = -1.
            print(ang)
            norm1 = sgn * cross(Z,vec1).unit()
            norm2 = sgn * cross(Z,vec2).unit()
            l1 = Line([Point([prev[0], prev[1] , 0.]), vec1])
            l2 = Line([Point([pt[0], pt[1], 0.]), vec2])
            ln1 = Line([Point([prev[0] + norm1[0] * rad, prev[1] + norm1[1] * rad, 0.]),
                        vec1])
                              
            
            ln2 = Line([Point([nex[0] + norm2[0] * rad, nex[1] + norm2[1] * rad, 0.]),
                        vec2])
            
            center_arc = intersect_2_lines(ln1, ln2)
            if center_arc:
                cen = [center_arc[j] for j in range(2)]
                lnorm1 = Line([center_arc, -norm1])
                lnorm2 = Line([center_arc, -norm2])
                start = intersect_2_lines(lnorm1,l1)
 
                end = intersect_2_lines(lnorm2,l2)
 
                vecstart = Vector([start[i]-tmppt[i] for i in range(3)])
                vecend = Vector([end[i]-tmppt[i] for i in range(3)])
                if vecstart.norm > 0.5 * vec1.norm:
                    print('too long')
                    tmpvec = vecstart.unit()
                    start = Point([tmppt[i] + 0.5 * vec1.norm * tmpvec[i] for i in range(3)])
                if vecend.norm > 0.5 * vec2.norm:
                    tmpvec = vecend.unit()
                    end = Point([tmppt[i] + 0.5 * vec2.norm * tmpvec[i] for i in range(3)])
                bezier_parts[-1].append([start[0], start[1]])
                bezier_parts.append([[start[0], start[1]],[pt[0], pt[1]], [end[0], end[1]]])
                bezier_parts.append([[end[0], end[1]]])
                #plt.scatter([start[0], end[0]], [start[1], end[1]], c='g')
            else:
                cen = pt
                bezier_parts[-1].append([pt[0], pt[1]])
                bezier_parts.append([[pt[0], pt[1]]])
 
            #plt.scatter(cen[0], cen[1], c = 'r')
 
            prev = pt
        bezier_parts[-1].append([last[0], last[1]])
 
        polx = []
        poly = []
        last_point = bezier_parts[-1][-1]
        llast_point = bezier_parts[-1][len(bezier_parts[-1])-2]
        dx = last_point[0] - llast_point[0]
        dy = last_point[1] - llast_point[1]
        new_last_point = [last_point[0] + 10.*dx, last_point[1] + 10.*dy]

        bezier_parts.append([last_point,new_last_point])
        print(bezier_parts)
        for bp in bezier_parts:
            polx+=[p[0] for p in bp]
            poly+=[p[1] for p in bp]
        #plt.plot(polx, poly)
        def func(parameter):
            ctpts = None
            for i,bp in enumerate(bezier_parts):
                #parameter = min(parameter, 1.001)
                #print(parameter,bp[0][0],bp[-1][0], len(bp))
                if (bp[0][0]<=parameter) and (parameter<=bp[-1][0]):
                    ctpts = bp
                    break
            s = (parameter-ctpts[0][0])/(ctpts[-1][0] - ctpts[0][0])
            return de_casteljau(ctpts, s)[1]
        N = 1000
        x = [float(i)/float(N-1) for i in range(N)] 
        fx = [func(s) for s in x]
        #plt.plot(x,fx, '.')
        #plt.show()
        return func
    else:
        return linear_function(start_value, end_value)
    

def de_boor(control_points, knots, parameter):
    if len(knots) >= len(control_points):
        print('knots : '+str(knots))
        print('control : '+str(control_points))

    
def chord2_function(p1, p2, p3):
    def fun(parameter):
        if parameter <= p1:
            return (p2-1.)*parameter/p1


def minimized_bezier_function(control_points, value):
    def fun(parameter):
        return min(de_casteljau(control_points,parameter)[1], value)
    return fun


def bezier_function(control_points):
    def fun(parameter):
        return de_casteljau(control_points,parameter)
    return fun



from geo_functions import cross, angle
from geometry.Vector import Vector
class BezierSegment(object):
    # on ly in 2D for now
    def __init__(self, pt1, pt2, n_segments):
        self.n_segments = n_segments
        pts = [pt1]
        self.vec = [pt2[i] - pt1[i] for i in range(len(pt1))]
        self.vecunit = Vector(self.vec+[0.]).unit()
        self.normal = cross([0.,0.,1.], self.vec+[0.]).unit()
        for i in range(n_segments-1):
            start = pts[-1]
            pts.append([start[j] + (float(1.)/float(n_segments))*self.vec[j] for j in range(len(pt1))])
        pts.append(pt2)
        self.control_points = pts
        self.curve = bezier_function(self.control_points)
        self.parameters = [0. for p in self.control_points[1:-1]]

    def set_parameters(self, array):
        if len(array)==len(self.control_points)-2:
            for ipar,par in enumerate(array):
                self.control_points[ipar+1] = [self.control_points[ipar+1][0]+par[0]*self.vecunit[0] + par[1]*self.normal[0],
                                               self.control_points[ipar+1][1]+par[0]*self.vecunit[1] + par[1]*self.normal[1]]

    def tangent(self, parameter, dl):
        pt1 = self.curve(parameter)+[0.]
        pt2 = self.curve(parameter+dl)+[0.]
        vec =  Vector([pt2[i] - pt1[i] for i in range(3)]).unit()
        return vec

    
    def length(self, n=100):
        lg = 0.
        prev = self.curve(0.)
        for i in range(1,n):
            par = float(i)/float(n-1)
            pt = self.curve(par)
            lg += np.sqrt((pt[0] - prev[0])**2. + (pt[1] - prev[1])**2.)
            prev = pt
        return lg


class ArrayOfBezierSegments(list):
    def __init__(self, array):
        self.array = array
        
    def update_parameters(self, lst):
        istart = 0
        for s in self.array:
            parameters = []
            for j in range(s.n_segments-1):
                parameters.append(lst[istart])
                istart += 1
            print(parameters)
            s.set_parameters(parameters)

    @property
    def residuals(self):
        residuals = []
        tangents = []
        lengthes = []
        for s in self.array:
            tgstart = s.tangent(0., 0.001)
            tgend = s.tangent(1., 0.001)
            tangents.append([tgstart, tgend])
            lengthes.append(s.length())
        for i in range(len(self.array)):
            if i==0:
                res = angle(tangents[i][1], tangents[i+1][0], plane_normal=Vector([0.,0.,1.]))
            elif i==1:
                res = angle(tangents[i][0], tangents[i-1][0], plane_normal=Vector([0.,0.,1.]))
            else:
                res = angle(tangents[i][1], tangents[i+1][0], plane_normal=Vector([0.,0.,1.]))+\
                        angle(tangents[i][0], tangents[i-1][0], plane_normal=Vector([0.,0.,1.]))
            res+=lengthes[i]
            residuals.append(res)
        return residuals











def chord_function(p1):
    pts = [[0., 1.], [p1, 1.], [0.5 * (p1 +1.), (.5 + p1) ], [1.1, 0.5]]
    import matplotlib.pyplot as plt
    #plt.plot([p[0] for p in pts], [p[1] for p in pts])
    #plt.axis('equal')
    #plt.show()
    return minimized_bezier_function(pts, 1.)


def Bernstein(n,k):
    coeff=spbinom(n,k)
    def _bpoly(x):
        return coeff*x**k*(1-x)**(n-k)
    return _bpoly


def Bezier(points, num=100):
    N=len(points)
    t=np.linspace(0,1,num=num)
    curve=np.zeros((num,2))
    for ii in range(N):
        curve+=np.outer(Bernstein(N-1,ii)(t),points[ii])
    return curve

#def fit_points(points, tol):

if __name__=='__main__':

    n = 40
    print(str(n)+'!')
    print(factorial(n))
    n = 7
    print('--------')
    print('pascal triangle level '+str(n))
    pascal_triangle(n)
    points = [[0.,0.], [0.,1.]]
    print('--------')
    print('bezier curve '+str(points))
    #import matplotlib.pyplot as plt
    #plt.scatter([p[0] for p in points],[p[1] for p in points])
    #plt.plot([p[0] for p in points],[p[1] for p in points])
    #plt.axis('equal')
    sol = []
    N = 1000
    for i in range(N):
        parameter = float(i)/float(N-1)
        sol.append(de_casteljau(points, parameter))
    #plt.plot([p[0] for p in sol],[p[1] for p in sol])
    #plt.show()
    #points = chord_function(0.5, 0.9, 1.) 
    #fun = bezier_function(points)
    fun = chord_function(.8)
    sol = []
    l_over_d = 5.
    for i in range(N):
        parameter = float(i)/float(N-1)
        sol.append([parameter*l_over_d,fun(parameter)])
    #plt.clf()
    #plt.axis('equal')
    #plt.scatter([p[0] for p in points],[p[1] for p in points])
    #plt.plot([p[0] for p in points],[p[1] for p in points])
    #plt.plot([p[0] for p in sol],[p[1] for p in sol])
    #plt.show()
    points = [[0.,0.], [0.5,3.14], [1., 0.]]
    segments = []
    #plt.clf()
    parameters = []
    nb_internal = 2
    import random
    for i in range(len(points)-1):
        segments.append(BezierSegment(points[i], points[i+1], nb_internal+1))
        for p in range(nb_internal):
            parameters.append([random.random(),random.random()])
    aobs = ArrayOfBezierSegments(segments)
    aobs.update_parameters(parameters)
    print(aobs.residuals)
    for i,s in enumerate(aobs.array):
        segment_parameters = parameters[i*nb_internal: (i+1)*nb_internal]
        #s.set_parameters(segment_parameters)
        #plt.scatter([p[0] for p in s.control_points], [p[1] for p in s.control_points])
        ncv = 100
        xy = []
        for i in range(ncv):
            par = float(i)/float(ncv-1)
            xy.append(s.curve(par))
        #plt.plot([p[0] for p in xy], [p[1] for p in xy])
    #plt.show()



