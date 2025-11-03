from .Point import Point
import sys
import numpy as np
from ..geo_functions import cross, is_on_plane
from .Vector import Vector
from .Frame import Frame
class Plane(list):
    """
    defines a plane 
    """
    def __init__(self, *largs):
        self.reflection_mat = None
        if len(*largs) == 4:
            super(Plane, self).__init__(*largs)
        elif (len(*largs) == 3) and ([type(largs[0][i]) for i in range(3)] == [Point,Point,Point]):
            A = largs[0][0]
            B = largs[0][1]
            C = largs[0][2]
            v0 = Vector([B[i] - A[i] for i in range(3)])
            v1 = Vector([C[i] - A[i] for i in range(3)])
            v = cross(v0,v1)
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])
        elif (len(*largs) == 2) and ([type(largs[0][i]) for i in range(2)] == [Point,Vector]):
            A = largs[0][0]
            v = largs[0][1]
            super(Plane, self).__init__([v[0], v[1], v[2], -( v[0]*A[0] + v[1]*A[1] + v[2]*A[2] )])

    @property
    def normal_vector(self):
        return Vector([self[i] for i in range(3)]).unit()

    def reflection_matrix(self):
        mat = np.zeros((4,4), dtype = np.float)
        a = self[0]
        b = self[1]
        c = self[2]
        d = self[3]
        mat[0][0] = 1. - 2.*a**2.
        mat[0][1] =  - 2.*a*b
        mat[0][2] =  - 2.*a*c
        mat[0][3] =  - 2.*a*d
        mat[1][0] = -2.*a*b
        mat[1][1] = 1. - 2.*b**2.
        mat[1][2] = -2.*b*c
        mat[1][3] = -2.*b*d
        mat[2][0] = -2. * a* c
        mat[2][1] = -2. * b*c
        mat[2][2] = 1. - 2. * c**2.
        mat[2][3] = -2. * c * d
        mat[3][0] = 0.
        mat[3][1] = 0.
        mat[3][2] = 0.
        mat[3][3] = 1.
        self.reflection_mat = mat

    def reflect_point(self, pt):
        vpt = [pt[i] for i in range(3)]+[1.]
        self.reflection_matrix()
        return Point([float(np.dot(self.reflection_mat, vpt)[i]) for i in range(3)])

    def reflect_vector(self, vec):
        vec4 = [vec[i] for i in range(3)]+[0.]
        self.reflection_matrix()
        return  Vector([float(np.dot(self.reflection_mat, vec4)[i]) for i in range(3)])

    def reflect_frame(self, frame):
        self.reflection_matrix()
        return Frame([self.reflect_point(frame[0]), 
                      self.reflect_vector(frame[1]).unit(), 
                      self.reflect_vector(frame[2]).unit(), 
                      self.reflect_vector(frame[3]).unit()])


    def add_to_ax(self, ax, size=1.0, color='blue', alpha=0.2):
        """
        Visualize this plane on a matplotlib 3D axis.
        Args:
            ax: Matplotlib 3D axis
            size: Size of the plane visualization (default: 1.0)
            color: Color for the plane (default: 'blue')
            alpha: Transparency of the plane (default: 0.2)
        """
        # Get the normal vector of the plane
        normal = self.normal_vector
        
        # Find two vectors in the plane that are perpendicular to each other and to the normal
        if abs(normal[0]) > abs(normal[1]):
            v1 = Vector([normal[2], 0, -normal[0]])
        else:
            v1 = Vector([0, -normal[2], normal[1]])
        v1 = v1.unit()
        v2 = cross(normal, v1).unit()
        
        # Find a point on the plane
        if abs(self[0]) > 1e-6:
            x = -self[3]/self[0]
            y = 0
            z = 0
        elif abs(self[1]) > 1e-6:
            x = 0
            y = -self[3]/self[1]
            z = 0
        else:
            x = 0
            y = 0
            z = -self[3]/self[2]
        point_on_plane = Point([x, y, z])
        
        # Create a grid on the plane
        u = np.linspace(-size, size, 2)
        v = np.linspace(-size, size, 2)
        U, V = np.meshgrid(u, v)
        
        # Convert to 3D coordinates
        X = point_on_plane[0] + U * v1[0] + V * v2[0]
        Y = point_on_plane[1] + U * v1[1] + V * v2[1]
        Z = point_on_plane[2] + U * v1[2] + V * v2[2]
        
        # Plot the plane using plot_surface
        ax.plot_surface(X, Y, Z, color=color, alpha=alpha, linewidth=0.2, antialiased=True)
        
        # Plot the normal vector from the center of the plane
        normal_scaled = normal * size * 0.5
        ax.quiver(
            point_on_plane[0], point_on_plane[1], point_on_plane[2],
            normal_scaled[0], normal_scaled[1], normal_scaled[2],
            color=color,
            arrow_length_ratio=0.3
        )