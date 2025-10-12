import numpy as np

# Default characteristic length for mesh generation
lcar = 0.1

class Point(list):
    """
    3D Point class representing a point in Cartesian coordinates.
    
    Inherits from list for efficient storage and manipulation.
    Provides operations for vector arithmetic with other points and vectors.
    """
    def __init__(self, *largs, **kwargs):
        """
        Initialize a Point object.
        
        Args:
            *largs: Variable length list of coordinates [x, y, z]
            **kwargs: Optional keyword arguments (reserved for future use)
            
        Note:
            Point coordinates are accessed via indexing: point[0], point[1], point[2]
            representing x, y, and z coordinates respectively
        """
        super(Point, self).__init__(*largs)

    def __add__(self, vec):
        """Add a vector to this point, returning a new point."""
        return Point([self[i] + vec[i] for i in range(3)])
    
    def to_array(self):
        """Convert the point to a numpy array."""
        return np.array(self)
    
    def __neg__(self):
        """Return the negation of this point."""
        return Point([-self[i] for i in range(len(self))])

    def __sub__(self, other):
        """
        Subtract another point or vector from this point.
        
        Returns:
            Point: If subtracting a point, returns the vector from other to self.
                  If subtracting a vector, returns a new point.
        """
        other = other.__neg__()
        return self + (-other)

    def express_in_frame_plane(self, frame):
        """
        Project this point onto a plane defined by a frame and express its coordinates
        in the 2D coordinate system of that plane.
        
        Args:
            frame: A Frame object defining the plane and its coordinate system
            
        Returns:
            list: 2D coordinates [u, v] in the frame's plane
        """
        from ..geo_functions import dot
        from ..geometry.Plane import Plane
        from ..geometry.Frame import Frame
        from ..geometry.Vector import Vector
        from ..geo_functions import closest_point_on_plane
        
        # Extract basis vectors from the frame
        vecX = Vector(frame[2]).unit()  # X axis in the frame's plane
        vecY = Vector(frame[3]).unit()  # Y axis in the frame's plane
        origin = Point(frame[0])        # Origin of the frame
        
        # Create a plane from the frame's origin and normal
        plane = Plane([Point(frame[0]), Vector(frame[1])])
        
        # Project point onto the plane
        closest = closest_point_on_plane(self, plane)
        
        # Calculate 2D coordinates in the frame's coordinate system
        absc = dot(Vector([closest[i] - origin[i] for i in range(3)]), vecX)
        ordo = dot(Vector([closest[i] - origin[i] for i in range(3)]), vecY)
        
        return [absc, ordo]

    def pop_to_geom(self, geom):
        """
        Add this point to a geometry object (typically for mesh generation).
        
        Args:
            geom: A geometry object (e.g., from pygmsh)
            
        Returns:
            The point reference in the geometry
        """
        return geom.add_point(self, lcar)

    def add_to_ax(self, ax, color='k'):
        """
        Plot this point on a matplotlib 3D axis.
        
        Args:
            ax: Matplotlib 3D axis
            color: Color for the point (default: black)
        """
        ax.scatter(self[0], self[1], self[2], c=color, s=1)
