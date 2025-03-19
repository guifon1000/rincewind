import numpy as np
from geometry.Point import Point
from geometry.Frame import Frame
from geometry.Segment import Segment
from geometry.Vector import Vector
from geometry.Polyline3D import Polyline3D
from geo_functions import dot, cross
from analytic_functions.bezier_functions import piecewise_bezier_polyline, bezier_function
import types

class Polyline2D(list):    #always closed
    def __init__(self, *largs,**kwargs):
        super(Polyline2D,self).__init__(*largs)
        if 'closed' in kwargs:
            if kwargs['closed'] : 
                if not self.is_closed:
                    self.append(self[0])
        if 'z' in kwargs:
            self.z=kwargs['z']
        else:
            self.z = 0.
        self.pt3d = []
        if 'reference_point' in kwargs:
            self.reference_point = kwargs['reference_point']
        else:
            self.reference_point = [0., 0.]
        for i in range(len(self)):
            p = Point(self[i])
            self.pt3d.append( Point([0. , 0., 0.]))


    @property
    def is_closed(self):
        if self[0] == self[-1]:
            return True
        else:
            return False

    def close(self):
        if self.is_closed:
            print('the polyline is already closed')
        else:
            self.append(self[0])

    def to_frame(self, frame, **kwargs):
        #  ????? WHAT ???????????
        #         V
        # frame[1] is X
        # frame[2] is Y
        # frame[3] is Z, and the normal to the plane OXY
        try:
            fac_scale = kwargs['scale']
        except:
            fac_scale = 1.
        try:
            translate =  [kwargs['translate'][i] for i in range(2)]
        except:
            translate = [0., 0.]
        try:
            rotate =  kwargs['rotate']
        except:
            rotate = 0.


        pol3d = []
        origin = frame[0]
        basis = [frame[1], frame[2], frame[3]]
        rot_p = []
        if rotate != 0.:
            for p in self:
                v = Vector([p[0], p[1], 0.])
                rpx = dot(v, Vector([np.cos(rotate), np.sin(rotate), 0.]))
                rpy = dot(v, Vector([-np.sin(rotate), np.cos(rotate), 0.]))
                rot_p.append([rpx, rpy])
        else:
            rot_p=[[p[0], p[1]] for p in self]
        for p in rot_p:
            x3d = origin[0] + fac_scale * ( (p[0] - translate[0]) * dot(basis[1],(1.,0.,0.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(1.,0.,0.)))
            y3d = origin[1] + fac_scale * ( (p[0] - translate[0]) * dot(basis[1],(0.,1.,0.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(0.,1.,0.)))
            z3d = origin[2] + fac_scale * ( (p[0] - translate[1]) * dot(basis[1],(0.,0.,1.)) +
                                             (p[1] - translate[1]) * dot(basis[2],(0.,0.,1.)))
            pol3d.append(Point([x3d, y3d, z3d]))
        return Polyline3D(pol3d)


    def pop_to_geom(self, geom):
        pts = []
        lns = []
        if self.is_closed:
            for i,p in enumerate(self[:-1]):
                p = geom.add_point((p[0], p[1], 0.),0.1)
                pts.append(p)
            for i in range(len(pts)-1):
                l = geom.add_line(pts[i],pts[i+1])
                lns.append(l)
           
            l = geom.add_line(pts[-1], pts[0])
            lns.append(l)

        else:
            for i,p in enumerate(self):
                p = geom.add_point((p[0], p[1], 0.),0.1)
                pts.append(p)
            for i in range(len(pts)-1):
                l = geom.add_line(pts[i],pts[i+1])
                lns.append(l)
        return lns

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot([p[0] for p in self], [p[1] for p in self])
        plt.axis('equal')
        plt.show()

    def orient(self, thumb='outside'):
        if self.orientation!=thumb:
            super(Polyline2D, self).__init__(self[::-1])

    @property
    def orientation(self):
        from ..geo_functions import is_in_polyline
        if self.is_closed :
            p1 = Point([self[0][0], self[0][1], 0.])
            p2 = Point([self[1][0], self[1][1], 0.])
            vec_tangential = Vector([p2[i] - p1[i] for i in range(3)]).unit()
            vec_normal = Vector([0.,0.,1.])
            thumb_vec = cross(vec_tangential, vec_normal).unit()
            mid = Point([0.5 * (p1[i] + p2[i]) for i in range(3)])
            test_pt3d = Point([mid[i] + 0.001 * thumb_vec[i] for i in range(3)])
            test_pt = [test_pt3d[0], test_pt3d[1]]
            if is_in_polyline(test_pt, self) :
                return 'inside'
            else:
                return 'outside'




    @property
    def segments(self):
        segments = []
        frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        pol3d = self.to_frame(frame)
        for i,p in enumerate(pol3d[:-1]):
            pp1 = pol3d[i+1]
            segments.append(Segment([p, pp1]))
        return segments

    def auto_intersects(self):
        from ..geo_functions import intersect_2_segments #(seg1, seg2, strict = False):
        for i,si in enumerate(self.segments):
            for j,sj in enumerate(self.segments):
                if (i!=j) and (j!=i+1) and (j!=i-1):
                    if intersect_2_segments(si, sj, strict = False):
                        return True
        return False
		    


class Circle(Polyline2D):
    def __init__(self, center=[0., 0.], radius=1., nseg=20, closed=True):
        pts = []
        for i in range(nseg):
            teta = 2.*np.pi * float(i) /float(nseg)
            pts.append(
	        [center[0] + radius * np.cos(teta),
		center[1] + radius * np.sin(teta)]
		)
        super(Circle,self).__init__(pts, closed = closed) 


    
class NamedPolyline(dict):
    def __init__(self, pol2d, index_dict=None, ordered_keys=None, reverse=False, key='surface'):
        dico = {}
        if not index_dict and not ordered_keys:
            if reverse:
                dico[key] = Polyline2D(pol2d[::-1])
            else:
                dico[key] = Polyline2D(pol2d)
            self.ordered_keys=[key]
        elif ordered_keys and (len(pol2d)==len(ordered_keys)):
            self.ordered_keys = ordered_keys
            for i,k in enumerate(ordered_keys):
                dico[k]=pol2d[i]
        elif index_dict:
            self.ordered_keys=ordered_keys
            for k in index_dict:
                start = index_dict[k][0]
                end = index_dict[k][1]
                dico[k] = Polyline2D(pol2d[start:end])
        super(NamedPolyline, self).__init__(dico)

    @property
    def whole_polyline(self):
        pol = [] #[self[self.ordered_keys[0]][0]]
        for k in self.ordered_keys:
            for p in self[k]:
                if len(pol)==0: pol.append(p)
                elif p!=pol[-1]:pol.append(p)
        return Polyline2D(pol)

    @property
    def is_closed(self):
        return self.whole_polyline.is_closed

    @property
    def orientation(self):
        return self.whole_polyline.orientation

    def to_frame(self, frame, scale = None):
        from rincewind.geometry.Named3DSection import Named3DSection
        pols = []
        for k in self.ordered_keys:
            pols.append(self[k].to_frame(frame))
        return Named3DSection(pols, self.ordered_keys)

    def reverse(self):
        for k in self.ordered_keys:
            self[k] = self[k][::-1]
        self.ordered_keys=self.ordered_keys[::-1]

        """
        if self.has_key('holes'):
            for h in self['holes']:
                h.reverse()
        """

    def orient(self,thumb='outside'):
        if self.orientation!=thume: self.reverse()

    def plot(self):
        import matplotlib.pyplot as plt
        plt.axis('equal')
        for k in self.ordered_keys:
            x = [p[0] for p in self[k]]
            y = [p[1] for p in self[k]]
            plt.plot(x,y)
        plt.show()


class NamedCircle(NamedPolyline):
    def __init__(self, name, center=[0., 0.], radius=1., nseg=20, closed=True):
        cir = Circle(center=center, radius=radius, nseg=nseg, closed=closed)
        npt = len(cir)
        name_dict={
                'surface': [0,npt]
                }

        ordo = [name+'_'+'surface']
        super(NamedCircle, self).__init__( cir, index_dict=name_dict, ordered_keys=ordo, reverse=False)

class NamedCartesianBox(NamedPolyline):
    def __init__(self, name, xmin, xmax, ymin, ymax):
        rect = Polyline2D([[xmax,ymax], [xmin,ymax], [xmin,ymin], [xmax,ymin], [xmax,ymax]])
        name_dict = {'Ymax':[0,2], 'Xmin':[1,3], 'Ymin':[2,4], 'Xmax':[3,5]}
        ordo = [name+'_Ymax', name+'_Xmin', name+'_Ymin', name+'_Xmax']
        super(NamedCartesianBox, self).__init__( rect, index_dict=name_dict, ordered_keys=ordo, reverse=False)


class NamedProfile(NamedPolyline):
    def __init__(self, name='profile', flip = False, **kwargs):
        from rincewind.modelers.profiles.splineProfileMultiParam import Profile
        print(kwargs)
        profile = Profile(**kwargs) # creation of the 2d profile
        print(profile.polyline())
        pol = profile.polyline(closed = True)
        npt = len(pol)-1
        if flip == True:
            name_dict = {
                'intrados': [0, (npt-1)/2+1],
                'extrados': [(npt-1)/2, npt],
                'trailing_edge': [npt-1, npt+1]
                }
            ordo = [name+'_'+'intrados',name+'_'+'extrados',name+'_'+'trailing_edge']
        else:
            name_dict = {
                'extrados': [0, (npt-1)/2+1],
                'intrados': [(npt-1)/2, npt],
                'trailing_edge': [npt-1, npt+1]
                }
            ordo = [name+'_'+'extrados',name+'_'+'intrados',name+'_'+'trailing_edge']
        super(NamedProfile, self).__init__( pol, index_dict=name_dict, ordered_keys=ordo, reverse=False)

    

class parametric_curve_2d(object):
    def __init__(self, fX, fY, name = ''):
        #if isinstance(fX, types.FunctionType):
        self.xfunc = fX
        #elif (type(fX)==list) :
        #    self.xfunc = lambda s: fX[0] + s*(fX[1]-fX[0])
        #if isinstance(fY, types.FunctionType):
        self.yfunc = fY
        #elif (type(fY)==list) :
            #self.yfunc = piecewise_bezier_polyline(fY[0], fY[1])
        #    self.yfunc = lambda s: fY[0] + s*(fY[1]-fY[0])
        self.fct = lambda parameter: [self.xfunc(parameter), self.yfunc(parameter)]
        if name:self.set_name(name)
        else:self.name = None

    def set_name(self, name):
        self.name = name

    def tangent_vector(self,s, eps=0.001, unit=True):
        if s==1.:
            s-=eps
        vec = [self.fct(s+eps)[i]-self.fct(s)[i] for i in range(2)]
        norm = np.sqrt(vec[0]**2 + vec[1]**2)
        if unit:
            return [vec[i]/norm for i in range(2)]
        else:
            return [vec[i] for i in range(2)]

    def is_linear(self, N=1000):
        dx = set()
        dy = set()
        for i in range(N):
            s = float(i)/float(N)
            s_plus_ds = s + 1./float(N)
            dx.add(self.xfunc(s_plus_ds) - self.xfunc(s))
            dy.add(self.yfunc(s_plus_ds) - self.yfunc(s))
        return (len(dx)==1 or len(dy)==1)

    def plot(self, N=100):
        import matplotlib.pyplot as plt
        x = []
        y = []
        for i in range(N):
            s = float(i)/float(N-1)
            x.append(self.fct(s)[0])
            y.append(self.fct(s)[1])
        #plt.clf()
        plt.axis('equal')
        plt.plot(x,y)
        plt.show()




class piecewise_function_2d(list):
    def __init__0(self, *largs):
        if [type(f) for f in largs] == [function_and_domain for f in largs]:
            super(piecewise_function, self).__init__(largs)

    def __init__(self, param_curves):
        last_value = [0., 0.]
        continuous = True
        last_parameter = 0.
        self.discontinuities = []
        self.array = []
        for f in param_curves:
            #print('------------')
            #print('f.fct(0.) ='+str(f.fct(0.)))
            #print('f.fct(1.) ='+str(f.fct(1.)))
            if len(self.array)==0:
                self.array.append(f)
            else:
                if f.fct(0.) != self.array[-1].fct(1.):
                    print('discontinuity '+str(f.fct(0.))+'=/='+str(self.array[-1].fct(1.)) )
                    f_link = parametric_curve_2d([self.array[-1].xfunc(1.), f.fct(0.)[0]],\
                                                 [self.array[-1].xfunc(1.), f.fct(0.)[1]])
                    self.array.append(f_link)
                self.array.append(f)
        idx_default = 0
        for f in self.array:
            if not f.name:
                f.set_name('default_'+str(idx_default))
                idx_default+=1
    
    def analyze_linearity(self):
        for i,f in enumerate(self.array):
            if f.is_linear():
                print('part # '+str(i)+' of the curve is linear')

    def plot(self):
        import matplotlib.pyplot as plt
        plt.clf()
        for f in self.array:
            if False:#f.is_linear():
                N = 3
            else:
                N=500
            x = []
            y = []
            for i in range(N):
                s = float(i)/float(N-1)
                xy = [f.xfunc(s), f.yfunc(s)]
                x.append(xy[0])
                y.append(xy[1])
            plt.scatter(x,y)
        plt.axis('equal')
        plt.show()

    def named_polyline(self, n=50):
        polylines = []
        ordered_keys = []
        for f in self.array:
            polyline = []
            N = 3
            if f.name:
                ordered_keys.append(f.name)
            else:
                sys.exit('a polyline has no name')
            if f.is_linear():
                N = 2
            else:
                N = n
            for i in range(N):
                s = float(i)/float(N-1)
                polyline.append(f.fct(s))
            polylines.append(Polyline2D(polyline))
        return NamedPolyline(polylines, ordered_keys=ordered_keys)

    def close(self):
        if not self.named_polyline().whole_polyline.is_closed:
            first = self.array[0].fct(0.)
            last = self.array[-1].fct(1.)
            ctpt = [first,last]
            fun = bezier_function(ctpt)
            self.array.append(parametric_curve_2d(lambda s: fun(s)[0], lambda s: fun(s)[1], name='closing'))
