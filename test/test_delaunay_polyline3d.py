from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
pol = Polyline3D([
    Point([0., 0., -0.25]),
    Point([0.25, 0.1, 0.2]),
    Point([0.6, 0.35, 0.1]),
    Point([0.4, 0.84, 0.]),
    Point([0.1, 0.4, -0.3]),
    Point([-0.83, 0.3, 0.]),
    Point([-0.4, -0.5, -0.]),
    Point([-0.9, 0., 0.0]),
    Point([-0.5, -0.25, 0.0]),
    Point([0., 0., -0.25])])

pol = []
import numpy as np
N=380
pl = []
R = 4.
for i in range(N):
    teta = 2.*np.pi*float(i)/float(N)
    r = R#*(1.+ (np.cos(teta)*np.sin(3.*teta))**2.)
    x = r*np.cos(teta)
    y = r*np.sin(teta)
    z = np.cos(teta)*np.sin(teta)
    pol.append(Point([x,y,z]))
pol.append(pol[0])

pol = Polyline3D(pol)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


pol.add_to_ax(ax)
tris = pol.triangulate_first(Vector([0.,0.,1.]))
for tri in tris:tri.add_to_ax(ax)

plt.show()
