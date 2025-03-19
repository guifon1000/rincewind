from rincewind.geo_functions import distance
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.geometry.Named3DSection import Named3DSection
from rincewind.geometry.Polyline2D import NamedPolyline, Polyline2D
from rincewind.geometry.TreeNode import TreeNode
from rincewind.geometry.Extrusion import Extrusion
from rincewind.geometry.ParametricCurve3D import ParametricCurve3D 
from dactylos.part import is_intersection, fuse_parts, subtract_parts, split_by_plane, write_solids 
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_MakeShell
from dactylos.rincewind_to_cascade.extrusion import fill
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.BRepTools import breptools_Write
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_xlim(-10.,10.)
#ax.set_ylim(-10.,10.)
#ax.set_zlim(-10.,10.)


h = 1.
l = 0.85
e = 0.2
eps = 0.00

pol1 = Polyline2D([
    [-eps ,0.5*(h+e)],
    [-eps + l+0.5*e,0.5*(h+e)],
    [-eps + l+0.5*e,-0.5*(h+e)],
    [-eps,-0.5*(h+e)],
    [-eps,-0.5*(h-e)],
    [-eps+l-0.5*e,-0.5*(h-e)],
    [-eps+l-0.5*e,0.5*(h-e)],
    [-eps,0.5*(h-e)],
    [-eps,0.5*(h+e)]
    ])

pol2 = Polyline2D([ [-p[0] , p[1] +(0.85*h+0.5*e) ] for p in pol1])
from rincewind.geo_functions import fuse_2_polylines, sew_2_polylines, segments_to_polylines, is_in_polyline, classify_polylines

polylines_2d, sewn = sew_2_polylines(pol1, pol2)


frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])

'''
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


    print pol3d.is_planar()
'''

classify_polylines(polylines_2d)

for pol in polylines_2d:
    pol3d = pol.to_frame(frame)
    pol3d.add_to_ax(ax)


from random import random

#N_points = 400
"""
for i in range(N_points):
    x = 3.*(random()-0.5)
    y = 5.*(random()-0.5)
    pt = Point([x,y,0.])
    col = None
    if is_in_polyline(pt, polylines_2d[0]):
        col = 'g'
    else:
        col = 'r'
    pt3d = Point([x,y,0.])
    ax.scatter(pt3d[0], pt3d[1], pt3d[2], c=col, s=1)
"""

    #tris = pol3d.triangulate_first([0.,0.,1.])
    #for t in tris:
    #    t.add_to_ax(ax)

#for s in seg_array:
#    s.add_to_ax(ax)

plt.show()
