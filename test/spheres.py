import sys
import os
# Add the parent directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from geometry.Frame import Frame 
from geometry.Point import Point 
from geometry.ParametricCurve3D import ParametricCurve3D 
from geometry.Vector import Vector
from geometry.TreeNode import TreeNode
from geometry.Polyline2D import NamedPolyline
from geometry.TreeNode import TreeNode
from geometry.Named3DSection import Named3DSection
from geometry.Extrusion import Extrusion
from geometry.Sphere import Sphere



from geo_functions import distance
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

house = json.load(open("./romarine_cad.json","r"))
data = {}
data['points'] = []
for p in house['points']:
    data['points'].append(Point(p))
data['walls'] = house['walls']


total_height = 3.
    
floor_frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])

extrusion_generator = ParametricCurve3D(
        lambda s: 0, lambda s: 0, lambda s: s ) 

polylines = []
all_tree_nodes = []
thick = 0.15
for ipoint,point in enumerate(data['points']):
    sector = [point]
    for iwall,wall in enumerate(data['walls']):
        if ipoint in wall:
            nex=None
            if wall[0]==ipoint:
                nex = data['points'][wall[1]]
            elif wall[1]==ipoint:
                nex = data['points'][wall[0]]
            length = distance(point, nex)
            vec = Vector([nex[i]-point[i] for i in range(3)]).unit()
            half = Point([point[i] + 0.5 * length * vec[i] for i in range(3)])
            sector.append([half, thick])
    all_tree_nodes.append(TreeNode([sector[0]] + [ secp[0] for secp in sector[1:]]))
    

part_array = []
fout = open("segments.txt","w")
for tn in all_tree_nodes:
    pol =  tn.offset(default_thickness = 0.2)
    named_pol = NamedPolyline(pol)
    print(named_pol)

    for p in pol[:-1]:
        nextp = pol[pol.index(p)+1]
        fout.write(    str(p[0])+" "+str(p[1])+" "+str(p[2]) +   " ; " +  str(nextp[0])+" "+str(nextp[1])+" "+str(nextp[2]) + "\n"  )
    #p = pol[-1]
    #nextp=pol[0]
    #fout.write(    str(p[0])+" "+str(p[1])+" "+str(p[2]) +   " ; " +  str(nextp[0])+" "+str(nextp[1])+" "+str(nextp[2]) + "\n"  )

    frame_array = extrusion_generator.frame_array_with_rmf(floor_frame,1)
    extrusion_list = []
    for frame in frame_array:
        named_3d_section_list = []
        for k in named_pol.keys():
            pol3d = named_pol[k].to_frame(frame)

            named_3d_section_list.append(pol3d)
        print(len(named_3d_section_list))
        named_3d_section = Named3DSection(named_3d_section_list, ['default'])
        extrusion_list.append(named_3d_section)
    #profile_array.append(
    #print(extrusion_list[0])
    ex = Extrusion('wall', extrusion_list, frame_array)

    #ex.classify_caps()



    ex.add_to_ax(ax)

    #part_array.append(fill(ex, patch_mode = 'polylines', polyline_mode='raw'))
fout.close()
plt.show()
pts = []
for tn in all_tree_nodes:
    pol =  tn.offset(default_thickness = 0.2)
    named_pol = NamedPolyline(pol)
    for p in named_pol['surface']:
        pts.append(p)

with open('ptcloud.txt','w') as fpts:
    for p in pts:
        fpts.write(str(p[0])+' '+str(p[1])+str(p[2])+'\n')
   
plt.show()

def slope(h0,sl,x):
    return h0+sl*x

x = []
y = []
N=10
for i in range(N):
    par = float(i)/float(N-1)
    x.append(par*2.09)
    y.append(slope(2,0.3,x[-1]))
plt.plot(x,y)
plt.show()

sp = Sphere()
sp.mercator_map()