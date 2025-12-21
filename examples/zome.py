from rincewind.geometry.CarpentryBars import Bar, BarPolygon
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Plane import Plane
from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.document.exporter import RincewindExporter
from rincewind.document.vtk_exporter import VTKExporter


import rincewind.document.gdl_parser as gdl_parser
import matplotlib.pyplot as plt
import json
from rincewind.modelers.solids import  create_rhomboid_zome

import numpy as np
  #aplomb=0.*Vector((0.15, -0.25, -0.8)), 
bar = Bar(Point((0, 0, 0)), Point((1, 1, 1)), section=(0.1, 0.2), name="TestBar")

exporter = VTKExporter()
ctx = gdl_parser.GeometryContext()
parser = gdl_parser.JSONGDLParser()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#vertices, faces = create_refined_icosahedron(2,8.)
#vertices, faces = create_rhomboid_zome(2., 6)
hzome = 4.
vertices, faces = create_rhomboid_zome(1.,int(hzome))
fcorners = open("solid_corners.txt","w")
fjson = open("solid_geometry.json","w")

bars = []
rwexporter = RincewindExporter()
for face in faces[0:1]:   #just one face for testing
    revface = face#[::-1]    #reverse for refined icosahedron"
    polypts = [Point(vertices[idx]) for idx in revface]
    poly3d = Polyline3D(polypts)
    
    #rwexporter.add_polyline(poly3d, "name")
    #ctx.add_polyline(gdl_parser.GDLPolyline(polypts))
    #for p in poly3d:
        #ctx.add_point(gdl_parser.GDLPoint(x=p[0], y=p[1], z=p[2]))
        
    barpoly = BarPolygon(Point((0,0,hzome/2.0)), polypts)
    barpoly.create_end_cuts()
    print("barpoly created", barpoly)
    lattis_plane = Plane((vertices[revface[0]], vertices[revface[1]], vertices[revface[2]]))
    lattis_plane.add_to_ax(ax, size=3.0, color='cyan', alpha=0.3)
    for ibar,bar in enumerate(barpoly.bars):
        bar.add_lengthwise_split(lattis_plane)
        bar.add_to_ax(ax, color='brown', alpha=0.8)
        bars.append(bar)
        pts = bar.get_vertices()
        for i in range(6):
            poli = bar.get_face_polyline(i)
            #if poli.is_planar():
            rwexporter.add_srf_pt(poli)
            #rwexporter.add_polyline(poli)
        fcorners.write(str(bar.get_vertices_as_string()))
json.dump(rwexporter.to_dict(), open("zob.txt",'w'), indent = 2)

print("=================================")
s1 = json.dumps(rwexporter.to_dict(), indent=2)
print("stringified JSON")
#print(s1)
zob = parser.parse_file("zob.txt")

s2 = json.dumps(zob.to_json(), indent=2)


print("=================================")
print("re-parsed JSON")
#print(s2)
print("=================================")
print(s1==s2)
fjson.write(str(rwexporter.to_dict()))
fjson.close()

exporter.export_bars_to_vtk(bars, f"icosa_.vtk")

plt.axis('equal')
#plt.show()