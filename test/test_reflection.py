from rincewind.geometry.Plane import Plane
from rincewind.geometry.Triangle import Triangle
from rincewind.geometry.Point import Point
from rincewind.geometry.Vector import Vector
from rincewind.geometry.Frame import Frame
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10.,10.)
ax.set_ylim(-10.,10.)
ax.set_zlim(-10.,10.)
#plt.axis('equal')

ptplane = Point([8.,1.,1.])
vecplane = Vector([5.,2.,-1.]).unit()

A = Point([-1.,-1.,0.])
B = Point([1.,5.,0.5])
C = Point([0.75,1.,-1.])
frame = Frame([B, Vector([1.,0.,0.]) , Vector([0.,1.,0.]), Vector([0.,0.,1.])])


ax.scatter(ptplane[0], ptplane[1], ptplane[2], c= 'g')

plane = Plane([ptplane, vecplane])
frame2 = plane.reflect_frame(frame)
frame.add_to_ax(ax)
frame2.add_to_ax(ax)





tr1 = Triangle([A,B,C])
refltri = []
for p in [A,B,C]:
    break
    ax.scatter(p[0], p[1], p[2], c='g')
    reflect = plane.reflect_point(p)
    refltri.append(reflect)
    ax.scatter(reflect[0], reflect[1], reflect[2], c='r')


trirefl = Triangle(refltri)


#ax.scatter(ptplane[0], ptplane[1], ptplane[2], c='k')
vecplane.add_to_ax(ax, ptplane)


tr1.add_to_ax(ax)
#trirefl.add_to_ax(ax)

print (plane.reflection_mat)

plt.show()
