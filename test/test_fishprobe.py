import sys
import types
from rincewind.geometry.Point import Point
from rincewind.geometry.Plane import Plane
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Segment import Segment
from rincewind.geometry.Line import Line
from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.geometry.Triangulation import Triangulation, read_stl_file, read_msh_file, merge
from rincewind.geometry.Polyline2D import Polyline2D, NamedPolyline, parametric_curve_2d, piecewise_function_2d
from rincewind.geo_functions import intersect_2_planes, intersect_2_segments, \
        intersect_2_lines, is_on_plane, intersect_line_and_plane, distance, \
        cross, closest_point_on_line, barycenter, is_in_polyline
from rincewind.analytic_functions.bezier_functions import piecewise_bezier_polyline, bezier_function
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dactylos.part import Part, step_to_part, subtract_parts
from utils.gmsh_functions import part_to_stl

project_name = 'ifremer_marker'
internal_profiles = []


def create_mark(total_length, lnose, rnose, pn1, pn2, lstraight, lenlarg, pe1, pe2, r2, lfillet, pfillet, rtail):
    ltail = total_length - (lnose+lstraight+lenlarg+lfillet)

    x0 = -0.035
    x1 = x0 + lnose
    x2 = x1 + lstraight
    x3 = x2 + lenlarg
    x4 = x3 + lfillet
    x5 = x4 + ltail

    nose_function = bezier_function([[x0,0.], [x0,pn1 * rnose], [x0+pn2*(x1-x0),rnose], [x1,rnose]])
    nose = parametric_curve_2d(lambda s: nose_function(s)[0], lambda s: nose_function(s)[1])

    straight1 = parametric_curve_2d([x1, x2], [rnose, rnose])


    enlarg_function = bezier_function([[x2,rnose],[x2+pe1*lenlarg,rnose], [x2+pe1*lenlarg,rnose], [x2+pe1*lenlarg+0.5*(x3-x2-pe1*lenlarg), (1.+pe2)*r2],[x3,r2]])
    enlarg = parametric_curve_2d(lambda s: enlarg_function(s)[0], lambda s: enlarg_function(s)[1])

    tang = enlarg.tangent_vector(1.)
    p1 = [x3,r2]
    p2 = [x3 + pfillet*lfillet, r2 + lfillet*pfillet*tang[1]/tang[0]]
    p3 = [x4, rtail]

    fillet_function = bezier_function([p1,p2,p3])
    fillet = parametric_curve_2d(lambda s: fillet_function(s)[0], lambda s: fillet_function(s)[1])

    #straight2 = parametric_curve_2d([x4,x4], [p3[1], rtail])

    straight3 = parametric_curve_2d([x4,x5], [rtail, rtail])

    straight4 = parametric_curve_2d([x5,x5], [rtail, 0.])

    f = piecewise_function_2d([nose, straight1, enlarg, fillet, straight3, straight4])
    f.close()
    return f

def internal_parts_intersection(f, internal_profiles):
    n = len(internal_profiles)
    intersect = [False]*n
    into = []
    outo = []
    for i,p in enumerate(internal_profiles):
        for pt in p:
            if not is_in_polyline(pt, f.named_polyline().whole_polyline):
                outo.append(pt)
                intersect[i] = True
                #break
            else:
                into.append(pt)
    return intersect


tri = read_stl_file('assets/cas2')

orig = Point([0.,0.,0.])
xdir = Vector([1.,0.,0.]).unit()
zdir = Vector([0.,0.,1.]).unit()
plane_normal = Vector([0.,1.,0.]).unit()
cutplane = Plane([orig, plane_normal])
trilist = tri.triangle_list

cloud = tri.section_by_plane(cutplane)

for p in cloud:
    plt.scatter(p[0], abs(p[2]),s=1,c='k')
plt.axis('equal')

d1 = d1_ref
d2 = d2_ref
total_length = 0.25
lnose = 0.012
rnose = 0.017
pn1 = 0.5
pn2 = 0.75
lstraight = 0.047
lenlarg = 0.12
pe1 = 0.5
pe2 = 0.8
r2 = 0.024
lfillet = 0.002
pfillet = 0.5
rtail = 0.01


dico = {
        'name': 'ifremer_marker',
        'cads':{
            'largueur.stp': {
                'cog': [0.0, 0.0, 0.00],
                'mass': 0.008,
                'dl': 0.0005
                },
            'carte.stp': {
                'cog': [0.07416872498473492,-0.0009495738000000019,-0.001560005959999998],
                'mass': 0.045,
                'dl': 0.001
                },
            'lest.stp': {
                'cog': [0.17, 0.0, 0.0],
                'mass': 0.1,
                'dl': 0.001}
            }
        }
            

project_name = dico['name']
epsilon = 0.00001
points_and_masses = []

internal_profiles = []
internal_parts = []

for pt in [cad for cad in dico['cads'].keys()]:
    dl = dico['cads'][pt]['dl']
    cog = Point(dico['cads'][pt]['cog'])
    mass = dico['cads'][pt]['mass']
    part = step_to_part(pt, pt.replace('.stp',''))
    vec_translation=[0.,0.,0.]
    if 'lest' in pt:
        vec_translation= [d1+d2,0.,0.]
    elif 'carte' in pt:
        vec_translation = [d1,0.,0.]
    tri = part.get_triangulation(project_name, dl=dl,read=True)
    cog = [cog[i] + vec_translation[i] for i in range(3)]
    tri.translate(vec_translation)
    part = part.translate(vec_translation)
    tri.write_stl_file(pt.replace('.stp',''))
    bds = tri.bounding_box
    profile = [[bds[0],epsilon]]
    rev_line = Line([Point([0.,0.,0.]), Vector([1.,0.,0.])])
    nslices = 20
    for j in range(nslices):
        par = float(j)/float(nslices-1)
        xslice = bds[0] + epsilon + par * (bds[3]-bds[0]-2.*epsilon)
        #print xslice
        plane = Plane([Point([xslice, 0., 0.]), Vector([1., 0., 0.])])
        cloud_part = tri.section_by_plane(plane)
        dmax = 0.
        for p in cloud_part:
            if distance(p,closest_point_on_line(p,rev_line))> dmax:
                dmax = distance(p,closest_point_on_line(p,rev_line))
        profile.append([xslice,dmax])
    profile.append([bds[3], epsilon])
    profile.append(profile[0])
    profile = Polyline2D(profile)



    points_and_masses.append([cog,mass])
    #profile = part.maximal_distance_to_xaxis(bds[0], bds[3], 20)
    perimeter = 0.
    for i,p in enumerate(profile[:-1]):
        pp1 = profile[i+1]
        perimeter+=distance(p+[0.], pp1+[0.])
    eps = 0.001
    ztr0 = 0.
    ztr = eps



    internal_profiles.append(profile)
    internal_parts.append(part)
    plt.plot([p[0] for p in profile], [p[1] for p in profile], '-k')



plt.axis('equal')
# Nose

f = create_mark(total_length, lnose, rnose, pn1, pn2, lstraight, lenlarg, pe1, pe2, r2, lfillet, pfillet, rtail)
profile = f.named_polyline().whole_polyline

plt.plot([p[0] for p in f.named_polyline().whole_polyline],[p[1] for p in f.named_polyline().whole_polyline])
for p in cloud:
    plt.scatter(p[0], p[2],s=1,c='k')
plt.show()
for fn in f.array:
    if fn.is_linear():
        N = 23
    else:
        N= 45
    x = []
    y = []
    for i in range(N):
        s = float(i)/float(N-1)
        xy = fn.fct(s)
        x.append(xy[0])
        y.append(xy[1])
plt.show()
part = f.revolve('mark')
print ('volume by occ')
print (part.volume)


tri = part.get_triangulation(project_name,dl=0.005)
#for t in tri.triangle_list:
#    try:
#        t.add_to_ax(ax)
#    except:
#        pass
tri.write_stl_file('mark', separate_surfaces= False)
part_with_holes = part
part_with_holes.set_name('mark_with_holes')

for hole in internal_parts:
    part_with_holes = subtract_parts(part_with_holes, hole)
    part_with_holes.set_name('mark_with_holes')


foam_density = 500.

points_and_masses.append([Point(part_with_holes.cog),part_with_holes.volume*foam_density])
print (barycenter(points_and_masses))
#print part_with_holes.volume
#print part_with_holes.cog
print ('#######################################')
bary =  barycenter(points_and_masses)[0]
mass =  barycenter(points_and_masses)[1]

import meshmagick.hydrostatics as mh
from meshmagick.mesh import Mesh 
from meshmagick.mmio import write_VTK
import math
import numpy as np

faces_mesh = []
for face in tri['faces']:
    for f in tri['faces'][face]:
        faces_mesh.append([f[0]-1, f[1]-1, f[2]-1, f[0]-1]) # meshmagick counts the indices from 0

msh = Mesh(tri['vertices'], faces_mesh, name='zob')
hydro_cog = np.dot(msh.rotate_y(0.5*math.pi), bary)
print ('------- used mass -------')
print (0.001*mass)
print ('volume by mm')
print (msh.volume)
hydro = mh.Hydrostatics(msh,cog=hydro_cog, mass=0.001*mass)
print (hydro.gravity_center)
write_VTK("before_hydrostatics.vtk", hydro._vertices, hydro._faces)
try:
    hydro.equilibrate()
except:
    print ('could not solve hydrostatics')
write_VTK("after_hydrostatics.vtk", hydro._vertices, hydro._faces)
print ('final position of the gravity center : '+str(hydro.gravity_center))
print ('the mesh is at equilibrium : '+str(hydro.is_at_equilibrium()))
print ('the mesh is stable : '+str(hydro.isstable()))
print ('transversal metacentric height '+str(hydro.transversal_metacentric_height))
print ('longitudinal metacentric height '+str(hydro.longitudinal_metacentric_height))


from dactylos.primitives import box
domain =  box(xmin = -2., xmax = 8., ymin= -2., ymax = 2., zmin = -2., zmax=2.,
               xmin_name = 'Xmin',
               ymin_name = 'Ymin',
               zmin_name = 'Zmin',
               xmax_name = 'Xmax',
               ymax_name = 'Ymax',
               zmax_name = 'Zmax')
domain.set_name('box')
tri_box = domain.get_triangulation(project_name, dl=1. ,read=True)
final = merge(tri_box, tri)
final.write_fms_file(project_name+'/domain')




