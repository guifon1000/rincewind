from rincewind.io_functions import write_geo
from rincewind.analytic_functions.bezier_functions import piecewise_bezier_polyline
from rincewind.geo_functions import distance
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Polyline2D import Polyline2D, NamedProfile
from rincewind.geometry.Extrusion import Extrusion
from rincewind.geometry.ParametricCurve3D import Spline3D, ParametricCurve3D
from rincewind.modelers.profiles.splineProfileMultiParam import Profile
import pygmsh as pg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


geom = pg.built_in.Geometry()
ndisc = 80
name = 'wing'
fore_span = 1.
max_chord = 0.1
#profile = NamedProfile(name, typ='fon' , par=[0.72  ,0.4 , 0.09 , 0.02 , 0.008] ,npt = 56) # creation of the 2d profile
profile = Profile(typ = 'fon', lead_edge_thck=0.82  , thck_p1=0.15 , thck_p2=0.02 , cmb_p1=0.05 , cmb_p2=0.015,npt = 7)
#fun_generator_x = piecewise_bezier_polyline(0.,-0.05, [[0.1,-0.0,0.01],[0.25,-0.02,0.2]])
fun_generator_x = piecewise_bezier_polyline(0.,0.)
fun_generator_y = piecewise_bezier_polyline(0., fore_span)
#fun_generator_z = piecewise_bezier_polyline(-0.01,-0.05,[[0.88,-0.01,1.]] )
fun_generator_z = piecewise_bezier_polyline(-0.02,-0.05)
#fun_generator_z2 = lambda s: fun_generator_z(s) - 0.01
fun_scale = piecewise_bezier_polyline(1.,0.5)
#fun_scale = piecewise_bezier_polyline(1.,0.5 )
fun_chord = lambda s: fun_scale(s) * max_chord 
fore_rot_fun = piecewise_bezier_polyline(0.025,0.005)
wing_gen = ParametricCurve3D(fun_generator_x, fun_generator_y, fun_generator_z)
wing_gen.add_to_ax(ax)
frame_zero = Frame([Point([0.,0.,0.]), Vector([0.,1.,0.]), Vector([-1.,0.,0.]), Vector([0.,0.,1.])])
fore_wing = Extrusion(name, profile, wing_gen, frame_zero, scale = fun_chord, rotate = fore_rot_fun, n=55, ax=ax)
tris = fore_wing.triangulate()
for tri in tris:
    try:
        tri.add_to_ax(ax)
    except:
        pass
plt.show()
fore_wing.add_to_geom(geom)
write_geo('testex',geom)

