from ..geometry.Point import Point
from ..geometry.Vector import Vector
from ..geometry.Frame import Frame
#from ..geometry.Plane import Plane

from scipy import interpolate
import numpy as np

def cross(u,v):
    pv=[]
    pv.append(u[1]*v[2]-u[2]*v[1])
    pv.append(u[2]*v[0]-u[0]*v[2])
    pv.append(u[0]*v[1]-u[1]*v[0])
    vp=Vector(pv)
    return vp


def distance(p1, p2):
    return Vector([p2[i] - p1[i] for i in range(3) ]).norm
    

def barycenter(points_and_masses):
    sum_mi = sum([pm[1] for pm in points_and_masses])
    sum_tmp = [0., 0., 0.]
    for pm in points_and_masses:
        point = pm[0]
        mass = pm[1]
        for i in range(3):
            sum_tmp[i]+=mass*point[i]
    for i in range(3):
        sum_tmp[i]/=sum_mi
    return Point(sum_tmp), sum_mi

def dot(u,v):
    return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]


def angle(u,v, plane_normal = None):
    x = u.unit()
    y = v.unit()
    angle = np.arctan2(cross(x,y).norm,dot(x,y))
    #
    if plane_normal:
        if dot(cross(u,v), plane_normal)>0.:
            return angle
        else :
            return - angle
    else:
        return angle
  
    # warning : only in 2D case
    #if dot(cross(u,v), Vector((0.,0.,1.0)))>0.:
    #    return angle
    #else :
    #    print("gne")
    #    return -angle


def intersect_3_planes(p1,p2,p3):
    A=np.array([p1[:3], p2[:3], p3[:3]])
    b=np.array([-p1[3],-p2[3],-p3[3]]).transpose()
    sol=np.linalg.solve(A, b)
    return Point([sol[i] for i in range(3)])


def get_parameter(pt, line):
    if is_on_line(pt, line):
        imax = np.argmax([np.abs(c) for c in line[1]])
        return (pt[imax]-line[0][imax])/line[1][imax]
    else:
        return None

def intersect_2_planes(pl1, pl2):
    from ..geometry.Plane import Plane
    from ..geometry.Line import Line
    n1 = pl1.normal_vector
    n2 = pl2.normal_vector
    line_dir = cross(n1,n2).unit()
    abs_line_dir = Vector([abs(c) for c in line_dir])
    imax =  np.argmax(abs_line_dir)
    pt = Point([0. for i in range(3)])
    pl3 = Plane([pt, line_dir])
    ptl = intersect_3_planes(pl1, pl2, pl3)
    return Line([ptl, line_dir])

def intersect_2_lines(l1, l2):
    from ..geometry.Plane import Plane
    v1 = l1[1]
    v2 = l2[1]
    normal_vect = cross(l1[1], l2[1])
    intersection = None
    if normal_vect.norm > 1.e-16:
        unit_normal = normal_vect.unit()
        components = [c**2. for c in unit_normal]
        # np.argpartition(components, -2) gives the array of indices of growing component 
        # (from smaller to biggest)
        # ind = np.argpartition(components, -2)[:2] gives the indices of the 2 smallest 
        # components of the vector
        ind = np.argpartition(components, -2)[:2]

        plane_1 = Plane((l1[0], unit_normal))

        if is_on_plane(l2[0], plane_1):
            i1 = ind[0]
            i2 = ind[1]
            A = np.array([[-v1[i1], v2[i1]], [-v1[i2], v2[i2]]])
            b = np.array([l1[0][i1] - l2[0][i1], l1[0][i2] - l2[0][i2]]).transpose()
            #print(A)
            #print(b)
            try:
                sol=np.linalg.solve(A, b)
                #print(sol)
                intersection = l1.parameter_point(sol[0]) 
            except:
                print('zoblines')
    return intersection


def intersect_2_segments(seg1, seg2):
    from ..geometry.Line import Line
    v1 = Vector([seg1[1][i] - seg1[0][i] for i in range(3)]).unit()
    v2 = Vector([seg2[1][i] - seg2[0][i] for i in range(3)]).unit()
    l1 = Line([seg1[0], v1])
    l2 = Line([seg2[0], v2])
    seg1_parameters = [get_parameter(seg1[0], l1), get_parameter(seg1[1], l1)]
    seg2_parameters = [get_parameter(seg2[0], l2), get_parameter(seg2[1], l2)]
    

    
    intersection = intersect_2_lines(l1,l2)
    out = False
    print('toto')
    print(is_on_line(seg1[0], l1))
    print(is_on_line(seg1[1], l1))
    print(is_on_line(seg2[0], l2))
    print(is_on_line(seg2[1], l2))
    
    print('zob')
    print(seg1_parameters)
    print(seg2_parameters)
    print(get_parameter(intersection, l1))
    print(get_parameter(intersection, l2))
    if intersection and ( seg1_parameters[0] <= get_parameter(intersection, l1) <= seg1_parameters[1]) and \
            ( seg2_parameters[0] <= get_parameter(intersection, l2) <= seg2_parameters[1]):
        out = True
    """
    elif ((distance(seg1[0], seg2[0])<1.e-12) or \
         (distance(seg1[1], seg2[0])<1.e-12) or \
         (distance(seg1[1], seg2[1])<1.e-12) or \
         (distance(seg1[0], seg2[1])<1.e-12)) :
        out=True
    """
    return out



def intersect_line_and_plane(line, plane):
    ptline = line[0]  # Point on the line
    vecline = line[1]  # Direction vector of the line
    normplane = plane[:3]  # Normal vector of the plane (a, b, c)
    d = plane[3]  # d in ax + by + cz + d = 0

    # Check if line and plane are parallel
    denominator = np.dot(vecline, normplane)
    if np.abs(denominator) < 1.e-18:
        return None  # Parallel or line lies on the plane

    # Calculate t for the intersection point
    numerator = -(np.dot(normplane, ptline) + d)
    t = numerator / denominator

    # Return the intersection point
    return [ptline[i] + t * vecline[i] for i in range(3)]


def intersect_line_and_plane000(line, plane):
    ptline = line[0]
    vecline = line[1]
    normplane = plane.normal_vector
    if np.abs(dot(vecline, normplane)) > 1.e-18:
        t =  -(plane[0]*ptline[0] + plane[1]*ptline[1] + plane[2]*ptline[2] + plane[3]) / dot(vecline, normplane) #warning : randomly added the +plane[3]
        return line.parameter_point(t)
    else:
        return None

def closest_point_on_plane(pt, plane):
    from ..geometry.Line import Line
    if is_on_plane(pt, plane):
        return pt
    else:
        line = Line([pt, plane.normal_vector])
        return intersect_line_and_plane(line, plane)

def closest_point_on_line(pt, line):
    #i.e. orthogonal projection of pt on line
    from ..geometry.Line import Line
    v1 = Vector([pt[i] - line[0][i] for i in range(3)])
    t = dot(v1, line[1])/line[1].norm**2.
    return line.parameter_point(t)


def center_arc(pt1, pt2, bulge):
    if bulge > 0.:
        inc_angle = 4. * np.arctan(bulge)
    elif bulge < 0.:
        inc_angle = -4. * np.arctan(bulge)
    chord = Vector([pt2[i] - pt1[i] for i in range(3)])
    mid = Point([0.5 * (pt1[i] + pt2[i]) for i in range(3) ])
    vec = (chord.norm * 0.5 * bulge * cross(chord, Vector((0., 0., 1.))).unit())
    summit = Point([mid[i] + vec[i] for i in range(3) ])
    radius = chord.norm / (2. * np.sin(inc_angle/2.))
    vec = radius * Vector([mid[i] - summit[i] for i in range(3)]).unit()
    center = Point([summit[i] + vec[i] for i in range(3) ])
    return center

def circle_from_3_points(pt1, pt2, pt3):
    pass


def create_treenodes(points, walls, default_thickness=None):
    from ..geometry.TreeNode import TreeNode   
    all_tree_nodes = []

    if [len(wall)>=2 for wall in walls]!=[True for wall in walls] and (not default_thickness):
        thick_mode = 'no'
    if default_thickness or [len(wall)>=2 for wall in walls]==[True for wall in walls]:
        thick_mode = 'default'
    for ipoint,point in enumerate(points):
        sector = [point]
        if thick_mode == 'default':
            thicknesses = []
        else:
            thicknesses = None
        for iwall,wall in enumerate(walls):
            if ipoint in wall[0]:
                nex=None
                if wall[0][0]==ipoint:
                    nex = points[wall[0][1]]
                elif wall[0][1]==ipoint:
                    nex = points[wall[0][0]]
                length = distance(point, nex)
                vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
                half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
                sector.append(half)
                if thick_mode == 'default' :
                    if len(wall)>=2 and float(wall[1]):
                        thicknesses.append(float(wall[1]))
                    else:
                        thicknesses.append(default_thickness)
        if thicknesses:
            tn = TreeNode([secp for secp in sector], thick = thicknesses)
        else:
            tn = TreeNode([secp for secp in sector])
        all_tree_nodes.append(tn)
    return all_tree_nodes

def offset_treenode_array(all_tree_nodes, first_orientation='outside'):
    polylines = []
    for itreenode, treenode in enumerate(all_tree_nodes): 
        pl = treenode.offset()#default_thickness = 0.2)
        polylines.append(pl)
    sew_polyline_array(polylines)
    levels = organize_polylines(polylines)
    orientation = first_orientation
    for level in levels:
        for pol in levels[level]:
            pol.orient(thumb = orientation)
        if orientation=='outside':orientation='inside'
        elif orientation=='inside':orientation='outside'
    return levels



def sew_polyline_array(polylines):
    cont = True
    while cont :
        found = False
        to_remove = []
        to_add = []
        for i, poli in enumerate(polylines): 
            for j, polj in enumerate(polylines):
                if i!=j:
                    polij, sewn = sew_2_polylines(poli, polj)
                    if sewn:
                        found = True
                        to_remove.append(poli)
                        to_remove.append(polj)
                        for pol in polij:
                            to_add.append(pol)
                        break
            if found:
                break
 
        for pol in to_remove:
            while pol in polylines:polylines.remove(pol)
        for pol in to_add:
            polylines.append(pol)
        if not to_remove:
            cont = False


def is_in_polyline(pt, pol):
    # the point and the polyline must be 2d
    from ..geometry.Segment import Segment
    big = 1.e6
    far_point = Point([big, 3.45485646*big,0.])
    number_of_intersections = 0
    pt3d = Point([pt[0], pt[1], 0.])
    seg1 = Segment([pt3d, far_point])
    for i in range(len(pol)-1):
        p0 = Point([pol[i][0], pol[i][1], 0.])
        p1 = Point([pol[i+1][0], pol[i+1][1], 0.])
        seg2 = Segment([p0, p1])
        if intersect_2_segments(seg1, seg2):
            number_of_intersections += 1
    return number_of_intersections%2 == 1

def is_polyline_in_polyline(pol1, pol2):
    from ..geometry.Polyline2D import NamedPolyline, Polyline2D

    if type(pol1)==Polyline2D:
        tpol1=pol1
    elif type(pol1)==NamedPolyline:
        tpol1=pol1.whole_polyline
    if type(pol2)==Polyline2D:
        tpol2=pol2
    elif type(pol2)==NamedPolyline:
        tpol2=pol2.whole_polyline
    print('=====================')
    print(tpol1)
    print('=====================')
    print(tpol2)
    print('=====================')
    #print([is_in_polyline(pt, tpol2) for pt in tpol1])
    return [is_in_polyline(pt, tpol2) for pt in tpol1] == [True for pt in tpol1]


def classify_polylines(polylines):
    tab = [[[] for pi in polylines] for pj in polylines]
    included = [[] for pi in polylines]
    containers = [[] for pi in polylines]
    for i,pi in enumerate(polylines):
        for j,pj in enumerate(polylines):
            if i!=j:
                print("comparing\n"+str(pi)+'\n<---->\n'+str(pj)+'\n')
                if is_polyline_in_polyline(pi,pj):
                    included[i].append(j)
                    containers[j].append(i)
    return included, containers
            
def organize_polylines(polylines):
    dico = {}
    ipass = 0
    while len(polylines)!=0:
        print('pass : '+str(ipass))
        included, containers = classify_polylines(polylines)
        to_remove = []
        for i in range(len(polylines)):
            if included[i] == []:
                if not dico.has_key('level_'+str(ipass)):
                    dico['level_'+str(ipass)] = []
                dico['level_'+str(ipass)].append(polylines[i])
        for pol in dico['level_'+str(ipass)]:
            polylines.remove(pol)
        ipass+=1
    return dico



def is_on_line(pt, line):
    c_ref = None
    s= None
    '''
    for i,coord in enumerate(line[1]):
        if coord != 0.:
            s = (pt[i] - line[0][i])/coord
            break
    '''
    i = np.argmax([abs(line[1][j]) for j in range(3)])
    s = (pt[i] - line[0][i])/line[1][i]
    pt_test = Point([line[0][i] + s * line[1][i] for i in range(3)])
    #if not s:
    #    print(line[1]
    #    1/0
    return distance(pt_test, pt)<1.e-6

def is_on_plane(pt, plane, tol=1.e-12):
    return (plane[0]*pt[0] + plane[1]*pt[1] + plane[2]*pt[2] + plane[3])**2. < tol


def classify_point_by_plane(pt,plane):
    if is_on_plane(pt,plane):
        return 'on'
    if plane[0]*pt[0] + plane[1]*pt[1] + plane[2]*pt[2] + plane[3] < 0.:
        return 'under'
    if plane[0]*pt[0] + plane[1]*pt[1] + plane[2]*pt[2] + plane[3] > 0.:
        return 'above'

def is_triangle_intersected_by_plane(triangle, plane):
    array = []
    for p in triangle:
        array.append(classify_point_by_plane(p,plane))
    if ('on' in array) or (('above') in array and ('under' in array)):
        return True
    else:
        return False

def cut_triangle_by_plane(triangle, plane, tol=1.e-12):
    from ..geometry.Line import Line
    from ..geometry.Segment import Segment
    from ..geometry.Polyline3D import Polyline3D
    if not is_triangle_intersected_by_plane(triangle, plane):
        return None
    else:
        frame = triangle.frame()
        p0 = triangle[0]
        p1 = triangle[1]
        p2 = triangle[2]
        l0 = Line([p0,p1])
        s0 = Segment([p0,p1])
        l1 = Line([p1,p2])
        s1 = Segment([p1,p2])
        l2 = Line([p2,p0])
        s2 = Segment([p2,p0])
        i0 = intersect_line_and_plane(l0, plane)
        i1 = intersect_line_and_plane(l1, plane)
        i2 = intersect_line_and_plane(l2, plane)
        pol = [p0]
        new_points = []
        if i0 and is_on_segment(i0, s0):
            if distance(i0,p0)>tol and distance(i0,p1)>tol:
                pol.append(i0)
                new_points.append(i0)
            elif distance(i0,p0)<tol:
                if p0 not in new_points:new_points.append(p0)
            elif distance(i0,p1)<tol:
                if p1 not in new_points:new_points.append(p1)
        pol.append(p1)
        if i1 and is_on_segment(i1, s1):
            if distance(i1,p1)>tol and distance(i1,p2)>tol:
                pol.append(i1)
                new_points.append(i1)
            elif distance(i1,p1)<tol:
                if p1 not in new_points:new_points.append(p1)
            elif distance(i1,p2)<tol:
                if p2 not in new_points:new_points.append(p2)
        pol.append(p2)
        if i2 and is_on_segment(i2, s2):
            if distance(i2,p2)>tol and distance(i2,p0)>tol:
                pol.append(i2)
                new_points.append(i2)
            elif distance(i2,p2)<tol:
                if p2 not in new_points:new_points.append(p2)
            elif distance(i2,p0)<tol:
                if p0 not in new_points:new_points.append(p0)
        pol.append(p0)
        polylines = []
        for i,npt in enumerate(new_points):
            if i==len(new_points)-1:
                nex= new_points[0]
            else:nex = new_points[i+1]
            polyline = []
            started = False
            to_remove = []
            ip = 0
            while True:
                p=pol[ip%(len(pol)-1)]   #only if the polyline is closed...
                if p==npt:
                    started=True
                if p==nex and started==True:
                    polyline.append(p)
                    break
                    started=False
                if started:
                    polyline.append(p)
                    #to_remove.append(p)
                ip+=1
            for p in to_remove:
                pol.remove(p)
            polyline.append(polyline[0])
            polylines.append(polyline)
        triangles = []
        for pol in polylines:
            tris = Polyline3D(pol).triangulate_planar(frame)
            for t in tris:triangles.append(t)
        return triangles
        #print([len(pol) for pol in polylines]
       

def triangulate_2_polylines(pol1, pol2):
    if len(pol1)!=len(pol2):
        return None
    else:
        triangulation = {}
        npt = len(pol1)
        for i in range(npt-1):
            first = []
            second = []

def bissector_plane(p0,p1):
    from ..geometry.Plane import Plane
    mid = Point([0.5 * (p0[i] + p1[i]) for i in range(3)])
    vec = Vector([p1[i] - p0[i] for i in range(3)]).unit()
    return Plane([mid, vec])

def rotation_minimized_frame(frame0, point1, tangvec1):
    pl1 = bissector_plane(frame0[0], point1)
    frame_left1 = pl1.reflect_frame(frame0)
    p1ml = Point([point1[i] + frame_left1[1][i] for i in range(3)])
    p1ptang = Point([point1[i] + tangvec1[i] for i in range(3)])
    pl2 = bissector_plane(p1ml, p1ptang)
    return pl2.reflect_frame(frame_left1)



def matrix_to_quaternion(m):
    from ..geometry.Quaternion import Quaternion
    diag=np.diag(m)
    np.append(diag,1.)

    tr= np.trace(m)+1.
    if tr>0.:
        s=0.5/np.sqrt(tr)
        x=(m[2,1]-m[1,2])*s
        y=(m[0,2]-m[2,0])*s
        z=(m[1,0]-m[0,1])*s
        w=0.25/s
        return Quaternion((w,x,y,z))




def parameter_frame(tck, s, mode = 'frenetb'):
    basis = []
    t =  interpolate.splev(s , tck, der = 0)
    orig = Point([t[0], t[1], t[2]])
    if mode == 'frenet':
        t =  interpolate.splev(s , tck, der = 1)
        xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
        t =  interpolate.splev(s , tck, der = 2)
        xpp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
        T = xp.unit()
        basis.append(T.unit())
        B = cross(xp, xpp)
        basis.append(B.unit())
        N = cross(B, T)
        basis.append(N.unit())

    if mode == 'Xnat':
        t =  interpolate.splev(s , tck, der = 1)
        xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
        T = xp.unit()
        basis.append(T)
        B = cross((1.,0.,0.), T)
        basis.append(B)
        N = cross(T, B)
        basis.append(N)

    if mode == 'Ynat':
        t =  interpolate.splev(s , tck, der = 1)
        xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
        T = xp.unit()
        basis.append(T)
        B = cross((0.,1.,0.), T)
        basis.append(B)
        N = cross(T, B)
        basis.append(N)


    if mode == 'Znat':
        t =  interpolate.splev(s , tck, der = 1)
        xp  = Vector( [float(t[0]), float(t[1]), float(t[2]) ])
        T = xp.unit()
        basis.append(T)
        B = cross((0.,0.,1.), T)
        basis.append(B)
        N = cross(T, B)
        basis.append(N)

    matrix = np.zeros((3,3), dtype =float)
    for i in range(3) : matrix[i] = basis[i]
    return Frame((orig, matrix))


def is_on_segment(pt, seg):
    line = seg.line()
    seg_parameters = sorted([get_parameter(seg[0], line), get_parameter(seg[1], line)])
    ans = False
    if is_on_line(pt, line):
        ptpar = get_parameter(pt, line)
        if (ptpar >= seg_parameters[0]) and (ptpar <= seg_parameters[1]):
            ans = True
    return ans

def correct_points_and_walls(data):
    # p1                   p2                         p3
    # ================= WALL 1 =========================
    #                      *
    #                      *
    #                      *
    #                   WALL 2
    #                      *
    #                      *

    #                      |
    #                     _|_
    #                     \ /
    #                      V

    # p1                   p2                         p3
    # ====== WALL 1 =======  ++++++++++ WALL 3 +++++++++
    #                      *
    #                      *
    #                      *
    #                   WALL 2
    #                      *
    #                      *
    from ..geometry.Segment import Segment
    from ..geometry.Point import Point
    segments = []
    for wall in data['walls']:
        seg = Segment([Point(data['points'][wall[0]]) , Point(data['points'][wall[1]])])
        segments.append(seg)

    cont = True
    while cont:
        to_remove = []
        to_add = []
        for p in data['points']:
            found = False
            pt = Point(p)
            for s in segments:
                if pt!=s[0] and pt!=s[1] and is_on_segment(pt,s):
                    found = True
                    to_remove.append(s)
                    to_add.append(Segment([s[0], pt]))
                    to_add.append(Segment([pt, s[1]]))
                    break
            if found:
                break
        else:
            cont=False
        for s in to_remove:segments.remove(s)
        for s in to_add:segments.append(s)
    walls = []
    for s in segments:
        walls.append([data['points'].index(s[0]), data['points'].index(s[1])])
    return {'points':data['points'], 'walls':walls}
def unify_2_segments(seg1, seg2, keep_seg2 = False):
    from ..geometry.Segment import Segment
    #print(seg1)
    #print(seg2)
    #print('----')
    #    s1[0]         s2[0]         s1[1]            s2[1]
    #      X---------------------------X
    #                    X------------------------------X
    #            s1'           s2'             s3'
    # ==>  X-------------X-------------X(--------------)X
    #                                           V
    #                                      if keep_seg2
    segs = []
    if seg1.line() == seg2.line():
        if (not is_on_segment(seg1[0], seg2) and not is_on_segment(seg1[1], seg2) and\
                not is_on_segment(seg2[0], seg1) and not is_on_segment(seg2[1], seg1)):
            if keep_seg2 == True : 
                segs = [seg1, seg2]
            else :
                segs = None
        else:
            line = seg1.line()
            #print(seg1)
            #print(seg2)
            parameters = sorted([get_parameter(seg1[0], line), get_parameter(seg1[1], line),
                get_parameter(seg2[0], line), get_parameter(seg2[1], line)])
            #while None in parameters:parameters.remove(None)
            #print(parameters)
            points = [line.parameter_point(par) for par in parameters]
            for i in range(len(points)-1):
                p = points[i]
                pp1 = points[i+1]
                if keep_seg2 == True :
                    segs.append(Segment([p,pp1]))
                else:
                    if is_on_segment(p,seg1) and is_on_segment(pp1, seg1):
                        segs.append(Segment([p,pp1]))
    else:
        segs = None
    return segs


def sew_2_polylines(pol1, pol2):
    from ..geometry.Polyline3D import Polyline3D
    from ..geometry.Polyline2D import Polyline2D
    from ..geometry.Frame import Frame
    frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    seg1 = [s for s in pol1.segments]
    seg2 = [s for s in pol2.segments]
    segs1 = []
    segs2 = []
    to_remove1 = []
    to_add1 = []
    to_remove2 = []
    to_add2 = []
    for s1 in seg1:
        for s2 in seg2:
            segar1 = unify_2_segments(s1,s2)
            segar2 = unify_2_segments(s2,s1)
            if segar1 :#and segar2:
                for s in segar1:
                    to_add1.append(s)
                to_remove1.append(s1)
            if segar2:
                for s in segar2:
                    to_add2.append(s)
                to_remove2.append(s2)
    for s1 in to_remove1:
        while s1 in seg1 : seg1.remove(s1)
    for s1 in to_add1:
        seg1.append(s1)
    for s2 in to_remove2:
        while s2 in seg2 : seg2.remove(s2)
    for s2 in to_add2:
        seg2.append(s2)
    seg_array = seg1 + seg2

    to_remove = []
    for s1 in seg1:
        if s1 in seg2:
            to_remove.append(s1)
    for s2 in seg2:
        if s2 in seg1:
            to_remove.append(s2)
    for s in to_remove:
        while s in seg_array: seg_array.remove(s)
    sewn = False
    if seg_array != seg1 + seg2:
        sewn = True
 
    polylines = []
    while len(seg_array)>0:
        segments_to_polylines(polylines, seg_array)

    polylines_2d = []

    for pol in polylines:
        pol3d = Polyline3D(pol)
        pol2d = []
        for p in pol3d:
            p2d = p.express_in_frame_plane(frame)
            pol2d.append(p2d)
        polylines_2d.append(Polyline2D(pol2d))   
    return polylines_2d, sewn

def segments_to_polylines(polylines,seg_array):
    finished = False
    points = []
    first = seg_array.pop(0)
    points.append(first[0])
    points.append(first[1])
    cont = True
    while cont:
        iseg = None
        reverse = None
        for i in range(len(seg_array)):
            seg = seg_array[i]
            if distance(seg[0], points[-1]) < 1.e-12:
                iseg=i
                reverse = False
                break
            elif distance(seg[1], points[-1]) < 1.e-12:
                iseg=i
                reverse = True
                break
        if iseg is not None:
            #print(iseg)
            next_segment = seg_array.pop(iseg)
            if reverse == False:
                points.append(next_segment[1])
            if reverse == True:
                points.append(next_segment[0])
        else:
            polylines.append(points)
            cont = False
    return polylines

    
    



def fuse_2_polylines(pol1, pol2):
    seg1 = pol1.segments
    seg2 = pol2.segments
    seg_array = []
    for s1 in seg1:
        app = True
        for s2 in seg2:
            if (s2==s1) or (s2==-s1):
                app = False
        if app:
            seg_array.append(s1)
    for s1 in seg2:
        app = True
        for s2 in seg1:
            if (s2==s1) or (s2==-s1):
                app = False
        if app:
            seg_array.append(s1)
    print('fuse')
    print(seg_array)
    return seg_array

    

def assembly_polylines3d(largs):
    # if we know several polylines can be assembled as one
    array = largs
    new_pol = array.pop(-1)
    cont = True
    while cont:
        #print(new_pol)
        if len(array) ==0: cont=False
        for i,pol in enumerate(array):
            if (pol[0] == new_pol[-1]):
                new_pol += array.pop(i)[1:]
            elif (pol[-1] == new_pol[0]):
                new_first_point = array.pop(i)[0]
                new_pol = [new_first_point] + new_pol
                #new_pol += array.pop(i)[::-1][1:]
            break
    return new_pol