from rincewind.geo_functions import distance
from rincewind.modelers.profiles.splineProfileMultiParam import Profile
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Plane import Plane
from rincewind.geometry.Point import Point
from rincewind.geometry.Frame import Frame
from rincewind.geometry.Polyline2D import Polyline2D
from rincewind.geometry.Spline3D import Spline3D
from rincewind.geometry.Extrusion import Extrusion
from rincewind.geometry.Segment import Segment
from rincewind.geometry.Polyline2D import Circle
from rincewind.geometry.ParametricCurve3D import ParametricCurve3D
from rincewind.analytic_functions.bezier_functions import piecewise_bezier_polyline
import pygmsh as pg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rincewind.io_functions import write_geo
import sys

geom = pg.built_in.Geometry()
def lin_fun(value1, value2):
    f = lambda x: value1 + (value2 - value1) * x
    return f


ndisc = 80
fore_span = 0.5
fore_max_chord = 0.12
fore_rot_fun = lin_fun(0.01, 1.)

fore_profile = Profile(typ = 'fon', lead_edge_thck=0.82  , thck_p1=0.15 , thck_p2=0.02 , cmb_p1=0.05 , cmb_p2=0.015,npt = 7) # creation of the 2d profile
fore_pol = fore_profile.polyline(closed = True)

fun_generator_x = piecewise_bezier_polyline(0.2,0.1,[[0.5,0.15,1.]])
fun_generator_y = piecewise_bezier_polyline(0.,1.,[[0.5,0.75,1.]])
fun_generator_z = piecewise_bezier_polyline(0.,0.1,[[0.85,0.,0.5]] )

wing_gen = ParametricCurve3D(fun_generator_x, fun_generator_y, fun_generator_z)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10.,10.)
ax.set_ylim(-10.,10.)
ax.set_zlim(-10.,10.)
wing_gen.add_to_ax(ax)

frame_zero = Frame([Point([0.,0.,0.]), Vector([0.,1.,0.]), Vector([-1.,0.,0.]), Vector([0.,0.,1.])])

frames = wing_gen.frame_array_with_rmf(frame_zero, n=2)
for f in frames:
    f.add_to_ax(ax, scale = 0.1)
plt.show()

if fore_pol.is_closed:
    open_fore_pol = fore_pol[:-1]
else:
    open_fore_pol = fore_pol
fore_npt = len(open_fore_pol)

fore_pol = {
        'extrados': Polyline2D(open_fore_pol[:(fore_npt-1)/2+1]),
        'intrados': Polyline2D(open_fore_pol[(fore_npt-1)/2:]),
        'trailing_edge': Polyline2D([open_fore_pol[-1], open_fore_pol[0]])
        }

fore_wing = Extrusion(fore_pol, wing_gen, frame_zero, scale = lambda s: 0.12, rotate = fore_rot_fun, n=3)
fore_wing.interpolate_polylines()
fore_wing.add_to_geom(geom)

write_geo('wing', geom)
