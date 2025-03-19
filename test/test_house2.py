from rincewind.geo_functions import distance
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.geometry.Named3DSection import Named3DSection
from rincewind.geometry.Polyline2D import NamedPolyline
from rincewind.geometry.TreeNode import TreeNode
from rincewind.geometry.Extrusion import Extrusion
from rincewind.geometry.ParametricCurve3D import ParametricCurve3D 
from dactylos.part import is_intersection, fuse_parts, subtract_parts, split_by_plane, write_solids 
from dactylos.rincewind_to_cascade.extrusion import fill
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_xlim(-10.,10.)
#ax.set_ylim(-10.,10.)
#ax.set_zlim(-10.,10.)
romarine = json.load(open('house.json','r'))
print romarine
data = {}
data['points'] = []
for p in romarine['points']:
    data['points'].append(Point(p))
data['walls'] = romarine['walls']


total_height = 3.
    
floor_frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])

total_height = 3.
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
for tn in all_tree_nodes:
    pol =  tn.offset(default_thickness = 0.2)
    named_pol = NamedPolyline(pol)
    print named_pol
    frame_array = extrusion_generator.frame_array_with_rmf(floor_frame,1)
    extrusion_list = []
    for frame in frame_array:
        named_3d_section_list = []
        for k in named_pol.keys():
            pol3d = named_pol[k].to_frame(frame)

            named_3d_section_list.append(pol3d)
        print len(named_3d_section_list)
        named_3d_section = Named3DSection(named_3d_section_list, ['default'])
        extrusion_list.append(named_3d_section)
    #profile_array.append(

    ex = Extrusion('wall', extrusion_list)
    ex.add_to_ax(ax)
    part_array.append(fill(ex, patch_mode = 'polylines', polyline_mode='raw'))
plt.show()

cont = True
new_array = []
while cont == True:
    found_intersection = False
    for i,si in enumerate(part_array):
        for j,sj in enumerate(part_array):
            if (i!=j) and is_intersection(si,sj):
                found_intersection = True
                part_array.remove(si)
                part_array.remove(sj)
                #part_array.append(fuse_parts(si,sj))
                new_array.append(fuse_parts(si,sj))
                print len(part_array)
                break
        if found_intersection:
            break
    if not found_intersection:
    #if  found_intersection:
        cont = False
part_array = new_array+ part_array
cont = True
new_array = []
while cont == True:
    found_intersection = False
    for i,si in enumerate(part_array):
        for j,sj in enumerate(part_array):
            if (i!=j) and is_intersection(si,sj):
                found_intersection = True
                part_array.remove(si)
                part_array.remove(sj)
                #part_array.append(fuse_parts(si,sj))
                new_array.append(fuse_parts(si,sj))
                print len(part_array)
                break
        if found_intersection:
            break
    if not found_intersection:
    #if  found_intersection:
        cont = False
part_array = new_array+ part_array
print len(part_array)
write_solids( "romarine", part_array, omit = [])
from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()
for part in part_array:
    display.DisplayShape(part.solid, color = 'GREEN')

display.FitAll()
start_display()
#for pt in parts : pt.occ_show()
#plt.show()
