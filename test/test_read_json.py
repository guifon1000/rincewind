import json
import sys
import matplotlib.pyplot as plt

from matplotlib.patches import PathPatch
from matplotlib.path import Path

from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.ParametricCurve3D import ParametricCurve3D
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Polyline2D import NamedPolyline
from rincewind.geometry.Extrusion import Extrusion
from rincewind.geo_functions import create_treenodes, offset_treenode_array, classify_polylines, cross
from dactylos.rincewind_to_cascade.extrusion import fill
from dactylos.part import write_solids, Part, sew_parts, is_intersection, subtract_parts
from utils.gmsh_functions import part_to_stl
from mpl_toolkits.mplot3d import Axes3D

fig3 = plt.figure()
#ax0 = fig3.add_subplot(131)
#ax0.axis('equal')
ax = fig3.add_subplot(121, projection='3d')
ax.axis('equal')
ax2 = fig3.add_subplot(122)
ax2.axis('equal')

data = json.load(open('assets/test_house.json', 'r'))
#data = json.load(open('assets/test_heater.json', 'r'))
#data = json.load(open('assets/test_2_sources.json', 'r'))
project_name = data['project_name']
Zvec = Vector([0.,0.,1.])

def view_pts2d(data,ax):
    for p in data['points_2d']:
        ax.scatter(p[1], p[2], c='k')
        ax.text(p[1], p[2], p[0])

def figure_2d():
    fg = plt.gca()
    #fg.set_xlim(-5.,5.)
    #fg.set_ylim(-5.,5.)
    fg.axis('equal')
    return fg

def add_pol_to_fig(pol, fig, col):
    plt.plot([p[0] for p in pol], [p[1] for p in pol], col)

def add_contour_and_holes_to_fig0(contour, holes, fig):
    add_pol_to_fig(contour, fig, 'r')
    for h in holes:
        add_pol_to_fig(h, fig, 'k')

def pathify(polyline):
    path = [Path.LINETO for p in polyline]
    path[0] = Path.MOVETO
    return path


def add_contour_and_holes_to_fig(contour, holes, fig):
    points = [p for p in contour]
    moves = pathify(contour)
    for hole in holes:
        points+=[ p for p in hole]
        for pth_hole in pathify(hole):
            moves.append(pth_hole)

    path = Path(points, moves)


    patch = PathPatch(path, alpha=0.5)
    fig.add_patch(patch)
    


def add_to_fig(array_2d_pol, fig):
    pols = []
    for pol in array_2d_pol:
        pols.append(pol.whole_polyline)
    included, containers = classify_polylines(pols)
    holes = []
    contours = []
    for i,cont in enumerate(containers):
        if cont:
            holes = [pols[icontained] for icontained in cont]
            contour = pols[i]
            add_contour_and_holes_to_fig(contour, holes, fig)


def create_roof(data, thickness=0.5):
    from dactylos.cad_functions import face_polyline3d
    faces = []
    parts = []
    keys = []
    for islope,slope in enumerate(data['roof']):
        dicoface = {}
        pol3d_start = []
        pol3d_end = []
        for p in slope:
            p2 = find_2d_point(data, p[0])
            pstart = Point([p2[0], p2[1], p[1]])
            pend = Point([p2[0], p2[1], p[1]+thickness])
            pol3d_start.append(pstart)
            pol3d_end.append(pend)
        if not pol3d_start[-1] == pol3d_start[0]:
            pol3d_start.append(pol3d_start[0])
        if not pol3d_end[-1] == pol3d_end[0]:
            pol3d_end.append(pol3d_end[0])
        keystart = 'roof_'+str(islope)+'_start'
        keyend = 'roof_'+str(islope)+'_end'
        dicoface[keystart] = [face_polyline3d(pol3d_start).Shape()]
        dicoface[keyend] = [face_polyline3d(pol3d_end).Shape()]
        for i,p1 in enumerate(pol3d_start[:-1]):
            polside = [p1]
            polside.append(pol3d_start[i+1])
            polside.append(pol3d_end[i+1])
            polside.append(pol3d_end[i])
            polside.append(p1)
            keyside = 'roof_'+str(islope)+'side_'+str(i)
            dicoface[keyside] = [face_polyline3d(polside).Shape()]
        parts.append(Part(dicoface))
    cont = True
    while cont:
        print (len(parts))
        for i,parti in enumerate(parts):
            found_intersection = False
            for j,partj in enumerate(parts):
                if i!=j:
                    if is_intersection(parti, partj):
                        found_intersection = True
                        parts.remove(parti)
                        parts.remove(partj)
                        parts.append(sew_parts(parti, partj))
                        break
            else:found_intersection = False
            if found_intersection:break
        else:break
    if len(parts)==1:
        parts[0].set_name("roof")
        return parts[0]
    else:
        for iroof,part in enumerate(parts):
            part.set_name('roof_'+str(iroof+1))
            print ('case with several roofs not implemented yet')
            sys.exit()
    return parts 
    



def find_2d_point(data, name):
    for p in data['points_2d']:
        if p[0] == name:
            return [float(p[1]), float(p[2])]
            break
    else:
        print ('point '+name+' not found, aborting')
        sys.exit()


def vertical_frame_array(start_height, height, ndisc=1):
    fun_generator_x = lambda s: 0.
    fun_generator_y = lambda s: 0.
    fun_generator_z = lambda s: height * s+start_height
    generator = ParametricCurve3D(fun_generator_x, fun_generator_y, fun_generator_z)
    floor_frame = Frame([[0.,0.,start_height], [0, 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    frame_array = generator.frame_array_with_rmf(floor_frame, n=ndisc)
    return frame_array

def through_wall_frame_array(data, hole):
    pts = hole[0].split('-')
    thickness = 0.
    hole_extrusion = None
    for k in data:
        if 'stage' in k:
            for k2 in data[k]:
                if 'wall' in k2:
                    for w in data[k][k2]:
                        if w[0]==hole[0] and w[1]>=thickness:
                            thickness = w[1]
                            print (" wall "+w[0]+" found with thickness "+str(thickness))
                            pts = w[0].split('-')
                            start = find_2d_point(data,pts[0]) 
                            end = find_2d_point(data,pts[1])
                            st3d = Point([start[0], start[1], 0.])
                            en3d = Point([end[0], end[1], 0.])
                            vecwall = Vector([en3d[i] - st3d[i] for i in range(3)]).unit()
                            vecnorm = cross(vecwall, Zvec)
                            stpt = Point([st3d[i] + hole[1] * vecwall[i] + hole[2] * Zvec[i] -0.505*thickness * vecnorm[i] for i in range(3) ])
                            endpt = Point([st3d[i] + hole[1] * vecwall[i] + hole[2] * Zvec[i] +0.505*thickness * vecnorm[i] for i in range(3) ])
                            fX = lambda s: stpt[0] + s*(endpt[0]-stpt[0])
                            fY = lambda s: stpt[1] + s*(endpt[1]-stpt[1])
                            fZ = lambda s: stpt[2] + s*(endpt[2]-stpt[2])
                            generator = ParametricCurve3D(fX, fY, fZ)
                            start_frame = Frame([stpt, vecnorm, vecwall, Zvec])
                            frame_array = generator.frame_array_with_rmf(start_frame, n=1)
                            pol2d = [[-0.5 * hole[3], 0.5*hole[4]]]
                            pol2d.append([0.5 * hole[3], 0.5*hole[4]])
                            pol2d.append([0.5 * hole[3], -0.5*hole[4]])
                            pol2d.append([-0.5 * hole[3], -0.5*hole[4]])
                            pol2d.append(pol2d[0])
                            hole_slice = NamedPolyline(pol2d, key = 'hole')
                            slices = populate_slices(frame_array, [hole_slice])
	#hole_extrusion = Extrusion('hole',slices,frame_array) ?????? 

    return hole_extrusion
                            

def create_huge_box(zstart, thickness, big=100.):
    frame_array = vertical_frame_array(zstart, thickness)
    pol = [[-big,big], [big,big], [big, -big], [-big, -big], [-big, big]]
    slices = populate_slices(frame_array, [NamedPolyline(pol)])
    ex = Extrusion('box', slices, frame_array)
    cad_box = fill(ex, patch_mode='polylines', polyline_mode='raw')
    return cad_box

def populate_slices(frame_array, all_polylines):
    slices = []
    for iframe, frame in enumerate(frame_array):
        parameter = float(iframe)/(float(len(frame_array)-1))
        polylines_on_frame = []
        print ('______________________________________')
        for a_slice in all_polylines:
            print (a_slice)
            #if 'lv0' in a_slice.ordered_keys[0]:
            sl = a_slice.to_frame(frame)
            polylines_on_frame.append(sl)
            #if 'lv1' in a_slice.ordered_keys[0]:
            #    sl = a_slice.to_frame(frame, scale = None)
            #    polylines_on_frame.append(sl)
        print (len(polylines_on_frame))
        slices.append(polylines_on_frame)
    return slices
    

part_array = []

parts_dict = {}
view_pts2d(data,ax2)

cad_ground = create_huge_box(-data['ground_layer_thickness'], data['ground_layer_thickness'])
cad_ground.set_name('ground')
part_to_stl(project_name, cad_ground, dl=1.)

parts_dict['ground'] = cad_ground

cad_uground = create_huge_box(-10., 10.-data['ground_layer_thickness'])
cad_uground.set_name('underground')
part_to_stl(project_name, cad_uground, dl=1.)
if data.has_key('roof'):
    roof_part = create_roof(data)
    part_array.append(roof_part)
    roof_cut = create_roof(data, 200.)
    roof_part.set_name('roof')
    part_to_stl(project_name, roof_part, dl=1.)
    parts_dict['roof'] = roof_part

#sys.exit()
#write_solids('roof', roof_parts)
from OCC.Display.SimpleGui import init_display
#display, start_display, add_menu, add_function_to_menu = init_display()

#for f in faces:
#    display.DisplayShape(f)
#start_display()

#sys.exit()
print ("#############################################")
print ("               HOUSE MODELER                 ")
print ("#############################################")
print (project_name)
stages = []
for k in data:
    if 'stage_' in k:
        stages.append(k)
stages = sorted(stages, key = lambda s: int(s.replace('stage_','')))

floor_level = data['ground_height']
cad_walls = []
cad_slabs = []
floor_polyline = None

cad_data = {}



for stage in stages:
    floor_level = data[stage]['floor']
    fig = plt.gca()
    if data[stage].has_key('external_walls'):
        stage_walls = []
        stage_points = []
        for wall in data[stage]['external_walls']:
            wall_name = wall[0]
            wall_thickness = wall[1]
            pts = [find_2d_point(data, wall_name.split('-')[i]) for i in range(2)]
            print ('the external wall '+ wall_name +' is between '+str(pts)+' and has thickness '+ str(wall_thickness))
            wall_idx = []
            for p in pts:
                pt3d = Point([p[0], p[1], 0.])
                if pt3d not in stage_points:
                    stage_points.append(pt3d)
                wall_idx.append(stage_points.index(pt3d))
            stage_walls.append([wall_idx, wall_thickness])
        all_tree_nodes = create_treenodes(stage_points, stage_walls)#, default_thickness = 0.2)
        levels = offset_treenode_array(all_tree_nodes, first_orientation = 'outside')
        all_polylines = []
        wall_levels = []
        idx_wall = 0

        while True:
            if 'level_0' in levels.keys() and not 'level_1' in levels.keys():
                wall_polyline = []
                nb = 0
                for contour in levels['level_'+str(idx_wall)]:
                    wall_polyline.append(NamedPolyline(contour, key=stage+'_lvl'+str(nb)))
                    nb+=1
                wall_levels.append(wall_polyline)
                break
            elif 'level_'+str(idx_wall) in levels.keys() and 'level_'+str(idx_wall+1) in levels.keys():
                wall_polyline = []
                nb = 0
                for contour in levels['level_'+str(idx_wall)]:
                    wall_polyline.append(NamedPolyline(contour, key=stage+'_lvl'+str(nb)))
                    nb+=1
                for contour in levels['level_'+str(idx_wall+1)]:
                    wall_polyline.append(NamedPolyline(contour, key=stage+'_lvl'+str(nb)))
                    nb+=1
                wall_levels.append(wall_polyline)
                idx_wall+=2
            else:
                break
        if type(data[stage]['height'])==float:
            height = float(data[stage]['height'])
        elif data[stage]['height']=='roof':
            height = 100.
""" 	wall_frames =vertical_frame_array(floor_level, height)
        i_level = 0
        for full_wall in wall_levels:
            add_to_fig(full_wall, ax2)
	    wall_slices = populate_slices(wall_frames, full_wall)
	    walls_extrusion = Extrusion(data[stage]['name'],wall_slices,wall_frames)
            walls_extrusion.add_to_ax(ax)
            cad_walls = fill(walls_extrusion, patch_mode='polylines', polyline_mode='raw')
            if data[stage]['height']=='roof':
                cad_walls = subtract_parts(cad_walls, roof_cut)
                cad_walls.set_name(data[stage]['name'])
            if data[stage].has_key('holes'):
                for hole in data[stage]['holes']:
                    hl = through_wall_frame_array(data, hole)
                    if hl:
                        cad_hl = fill(hl, patch_mode='polylines', polyline_mode='raw')
                        cad_walls = subtract_parts(cad_walls, cad_hl)
                        cad_walls.set_name(data[stage]['name'])
            if data[stage].has_key('dig'):
                if data[stage]['dig'][1] == 'plain':
                    cad_walls_filled = fill(walls_extrusion, patch_mode='polylines', polyline_mode='raw', with_holes=False)
                    if data[stage]['dig'][0]=='roof':
                        roof_part = subtract_parts(roof_part, cad_walls_filled)
                        roof_part.set_name('roof')
                        part_to_stl(project_name, roof_part, dl=1.)
                        parts_dict['roof'] = roof_part
            if data[stage].has_key('cap_start'):
                print('not implemented yet') """

            #sys.exit()
                
            #print cad_walls.name
            #sys.exit()
            #cad_walls_filled = fill(walls_extrusion, patch_mode='polylines', polyline_mode='raw', with_holes=False)
            #part_to_stl(project_name+'_'+str(i_level)+'_filled', cad_walls_filled)
            #part_to_stl(project_name, cad_walls, dl=1.) ???
            # parts_dict[data[stage]['name']] = cad_walls
            #part_array.append(cad_walls)
            #i_level+=1
        #floor_level+=data[stage]['height']

merge_parts = []
if data.has_key('merge'):
    for k in  data['merge']:
        merge_parts.append(parts_dict[k])

ax2.autoscale_view()
plt.show()
cont = True
while cont:
    print (len(merge_parts))
    for i,parti in enumerate(merge_parts):
        found_intersection = False
        for j,partj in enumerate(merge_parts):
            if i!=j:
                if is_intersection(parti, partj):
                    found_intersection = True
                    merge_parts.remove(parti)
                    merge_parts.remove(partj)
                    merge_parts.append(sew_parts(parti, partj))
                    break
        else:found_intersection = False
        if found_intersection:break
    else:break


print (len(merge_parts))
pt = merge_parts[0]
pt.set_name('merge')
part_to_stl(project_name,pt, dl=1.)
print (part_array)
write_solids('sew',part_array)
