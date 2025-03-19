from rincewind.geometry.Frame import Frame 
from rincewind.geometry.Extrusion import Extrusion 
from rincewind.geometry.ParametricCurve3D import ParametricCurve3D 
from rincewind.geometry.Polyline2D import NamedPolyline, Polyline2D
from rincewind.geometry.Named3DSection import Named3DSection
import json
import re
from rincewind.geo_functions import create_treenodes, offset_treenode_array, correct_points_and_walls, is_polyline_in_polyline, \
                                     classify_polylines
from dactylos.cad_functions import extrude_classified_polylines
from OCC.Display.SimpleGui import init_display
import sys
data = json.load(open('./assets/2_rooms_house.json', 'r'))
data = correct_points_and_walls(data)
s = json.dumps(data)
s = re.sub(r'\{','{\n', s)
s = re.sub(r',\s+\"',',\n\"', s)
s = re.sub(r'\}','\n}', s)
print s
sys.exit()
#data = json.load(open('./assets/4_walls.json', 'r'))
all_tree_nodes = create_treenodes(data['points'],data['walls'], default_thickness = 0.2)
levels = offset_treenode_array(all_tree_nodes, first_orientation = 'outside')
sorted_keys = sorted(levels.keys(), key = lambda s: int(s.replace('level_','')))

all_polylines = []
ilv0 = 0
ilv1 = 0
for contour in levels['level_0']:
    all_polylines.append(NamedPolyline(contour, key = 'lv0_'+str(ilv0)))
    ilv0+=1
for contour in levels['level_1']:
    all_polylines.append(NamedPolyline(contour, key = 'lv1_'+str(ilv1)))
    ilv1+=1

reference_classification = classify_polylines(all_polylines)

height=7.
ndisc = 5
fun_generator_x = lambda s: 0.
fun_generator_y = lambda s: 0.
fun_generator_z = lambda s: height * s
ext_scale = lambda s : -s**2. + s + 1.
int_scale = lambda s : 2.*s**2. -2.*s + 1.
# int_scale = lambda s : 4.*s**2. -3.*s + 1.    # to generate holes going out of contour (error)
generator = ParametricCurve3D(fun_generator_x, fun_generator_y, fun_generator_z)
floor_frame = Frame([[0.,0.,0.], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])
frame_array = generator.frame_array_with_rmf(floor_frame, n=ndisc)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10.,10.)
ax.set_ylim(-10.,10.)
ax.set_zlim(-10.,10.)

#a_slice = things[-1]
slices = []
frames = []
for iframe, frame in enumerate(frame_array):
    frames.append(frame)
    parameter = float(iframe)/(float(len(frame_array)-1))
    polylines_on_frame = []
    for a_slice in all_polylines:
        if 'lv0' in a_slice.ordered_keys[0]:
            sl = a_slice.to_frame(frame, scale = ext_scale(parameter))
            polylines_on_frame.append(sl)
        if 'lv1' in a_slice.ordered_keys[0]:
            sl = a_slice.to_frame(frame, scale = int_scale(parameter))
            polylines_on_frame.append(sl)
    slices.append(polylines_on_frame)

print '----------------------'
print len(set([len(sl) for sl in slices]))
named_3d_slices = []
for i,sl in enumerate(slices):
    print '$$$$$$$$ FRAME : '+ str(i)+ ' $$$$$$$$$$'
    polylines_in_slice = []
    for s in sl:
        polylines_in_slice.append(s.express_in_frame_plane(frames[i]))
    if classify_polylines(polylines_in_slice)== reference_classification:
        named_3d_slices.append(sl)

print len(named_3d_slices)
print len(frames)
#sys.exit()
"""
for sl in slices:
    pols = []
    ord_3d = []
    for k in sl.keys():
        ord_3d.append(k)
        pols.append(sl[k])
    named_3d_slices.append(Named3DSection(pols, ord_3d))
"""


ex = Extrusion('house', named_3d_slices, frames)

#print ex.ordered_keys
#sys.exit()

from dactylos.rincewind_to_cascade.extrusion import fill

cad = fill(ex, patch_mode='polylines', polyline_mode='raw')
from dactylos.part import write_solids, subtract_parts
from dactylos.primitives import box
partbox =  box(xmin = -10., xmax = 10., ymin= -10., ymax = 10., zmin = -10., zmax=10.,
               xmin_name = 'Xmin',
               ymin_name = 'Ymin',
               zmin_name = 'Zmin',
               xmax_name = 'Xmax',
               ymax_name = 'Ymax',
               zmax_name = 'Zmax')
edge_refinements = {}
for n in partbox.dicoface:
    if not edge_refinements.has_key(n):
        edge_refinements[n] = 1.
for part in [cad]:
    for n in part.dicoface:
        if not edge_refinements.has_key(n):
            edge_refinements[n] = 0.5

partbox = subtract_parts(partbox, cad)
write_solids('walls', [cad])
write_solids('walls_in_air', [partbox], **edge_refinements)
#cad.occ_show()
#plt.show()



slices = []

#for i in range(ndisc):
    #pf = things[0]
    #slices.append(things[0].to_frame

sys.exit()
solids = extrude_classified_polylines(levels, sorted_keys, floor_frame, height)
display, start_display, add_menu, add_function_to_menu = init_display()

for f in solids:
    if f[1] == 'wall':
        ais_shape = display.DisplayColoredShape(f[0], color='RED')
    elif f[1] == 'void':
        ais_shape = display.DisplayColoredShape(f[0], color='BLUE')
display.FitAll()
start_display()

colors = ['b', 'r', 'g', 'k', 'y']

import matplotlib.pyplot as plt
for level in levels:
    color = colors[int(level.replace('level_',''))]
    for pol in levels[level]:
        plt.plot([p[0] for p in pol], [p[1] for p in pol], color)
        for i,p in enumerate(pol[:-1]):
            plt.text(p[0], p[1], str(i), color = color)
plt.axis('equal')
plt.show()

