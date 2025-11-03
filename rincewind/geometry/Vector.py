import numpy as np


class Vector(list):
    """
    3D Vector class for geometric operations.
    
    Inherits from list for efficient storage and manipulation.
    Provides standard vector operations like addition, subtraction, scalar multiplication,
    normalization, and visualization.
    """
    def __init__(self, *largs):
        """
        Initialize a Vector object.
        
        Args:
            *largs: Variable length list of components [x, y, z]
        """
        super(Vector, self).__init__(*largs)

    def __rmul__(self, rval):
        """
        Scalar multiplication (right multiplication).
        
        Args:
            rval: Scalar value to multiply the vector by
            
        Returns:
            Vector: New vector with each component multiplied by rval
        """
        return Vector([rval * self[i] for i in range(len(self))])
    
    def __mul__(self, rval):
        """
        Scalar multiplication (left multiplication).
        
        Args:
            rval: Scalar value to multiply the vector by
            
        Returns:
            Vector: New vector with each component multiplied by rval
        """
        return self.__rmul__(rval)

    def __add__(self, other):
        """
        Vector addition.
        
        Args:
            other: Another vector to add to this one
            
        Returns:
            Vector: New vector as the sum of this vector and other
        """
        return Vector([self[i] + other[i] for i in range(len(self))])
    
    def __neg__(self):
        """
        Vector negation.
        
        Returns:
            Vector: New vector with all components negated
        """
        return Vector([-self[i] for i in range(len(self))])

    def __sub__(self, other):
        """
        Vector subtraction.
        
        Args:
            other: Vector to subtract from this one
            
        Returns:
            Vector: New vector representing this - other
        """
        other = other.__neg__()
        return self + (-other)

    @property
    def norm(self):
        """
        Calculate the Euclidean norm (length) of the vector.
        
        Returns:
            float: The length of the vector
        """
        return np.sqrt(self.norm2)

    @property
    def norm2(self):
        """
        Calculate the squared norm (squared length) of the vector.
        
        This is more efficient than norm when only comparing lengths,
        as it avoids the square root calculation.
        
        Returns:
            float: The squared length of the vector
        """
        return np.sum([c**2. for c in self])

    def unit(self):
        """
        Return a unit vector in the same direction as this vector.
        
        Returns:
            Vector: A normalized version of this vector with length 1
            
        Note:
            Commented code shows a previous implementation with a check for
            very small vectors, which might be useful if numerical stability
            issues arise.
        """
        #if self.norm > 1.e-28:
        return Vector([self[i]/self.norm for i in range(3)])
        #else:
        #    return Vector([0., 0., 0.])
    
    def add_to_ax(self, ax, pt, scale=1., color='red'):
        """
        Visualize this vector on a matplotlib 3D axis.
        
        Args:
            ax: Matplotlib 3D axis
            pt: Starting point of the vector (usually a Point object)
            scale: Scale factor to apply to the vector length (default: 1.0)
        """
        ax.quiver(
            pt[0], pt[1], pt[2],
            scale * self[0], scale * self[1], scale * self[2],
            length=1., normalize=False, color=color
        )

    
    def rotate_x(self, angle, degrees=True):
        """
        Rotate vector around X-axis.
         
        Args:
            angle: Rotation angle
            degrees: If True, angle is in degrees; if False, in radians (default: True)
        
        Returns:
            New rotated Vector
        """
        if degrees:
            angle = np.radians(angle)
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, cos_a, -sin_a],
            [0, sin_a, cos_a]
        ])
        
        rotated = rotation_matrix @ self
        return Vector(rotated)
    
    def rotate_y(self, angle, degrees=True):
        """
        Rotate vector around Y-axis.
        
        Args:
            angle: Rotation angle
            degrees: If True, angle is in degrees; if False, in radians (default: True)
        
        Returns:
            New rotated Vector
        """
        if degrees:
            angle = np.radians(angle)
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        rotation_matrix = np.array([
            [cos_a, 0, sin_a],
            [0, 1, 0],
            [-sin_a, 0, cos_a]
        ])
        
        rotated = rotation_matrix @ self
        return Vector(rotated)
    
    def rotate_z(self, angle, degrees=True):
        """
        Rotate vector around Z-axis.
        
        Args:
            angle: Rotation angle
            degrees: If True, angle is in degrees; if False, in radians (default: True)
        
        Returns:
            New rotated Vector
        """
        if degrees:
            angle = np.radians(angle)
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        rotation_matrix = np.array([
            [cos_a, -sin_a, 0],
            [sin_a, cos_a, 0],
            [0, 0, 1]
        ])
        
        rotated = rotation_matrix @ self
        return Vector(rotated)
    
    def rotate_axis(self, axis, angle, degrees=True):
        """
        Rotate vector around an arbitrary axis using Rodrigues' rotation formula.
        
        Args:
            axis: Vector or list representing the rotation axis
            angle: Rotation angle
            degrees: If True, angle is in degrees; if False, in radians (default: True)
        
        Returns:
            New rotated Vector
        """
        if degrees:
            angle = np.radians(angle)
        
        # Normalize the axis
        if isinstance(axis, Vector):
            k = axis.unit()
        else:
            k = np.array(axis)
            k = k / np.linalg.norm(k)
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Rodrigues' rotation formula
        # R = I + sin(θ)K + (1-cos(θ))K²
        # where K is the cross-product matrix of the axis
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        
        rotation_matrix = (np.eye(3) + sin_a * K + 
                          (1 - cos_a) * (K @ K))
        
        rotated = rotation_matrix @ self
        return Vector(rotated)
    
    def rotate_euler(self, roll, pitch, yaw, degrees=True, order='xyz'):
        """
        Rotate vector using Euler angles.
        
        Args:
            roll: Rotation around X-axis
            pitch: Rotation around Y-axis
            yaw: Rotation around Z-axis
            degrees: If True, angles are in degrees; if False, in radians (default: True)
            order: Order of rotations (default: 'xyz')
        
        Returns:
            New rotated Vector
        """
        if degrees:
            roll = np.radians(roll)
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)
        
        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Apply rotations in specified order
        rotation_order = {'x': Rx, 'y': Ry, 'z': Rz}
        rotation_matrix = np.eye(3)
        
        for axis in order.lower():
            rotation_matrix = rotation_order[axis] @ rotation_matrix
        
        rotated = rotation_matrix @ self
        return Vector(rotated)
    
    def rotate_to_align(self, target):
        """
        Find rotation that aligns this vector with target vector.
        
        Args:
            target: Target Vector or array to align to
        
        Returns:
            Tuple of (axis, angle) for the rotation
        """
        if isinstance(target, Vector):
            target = target
        else:
            target = np.array(target)
        
        # Normalize both vectors
        v1 = self.unit()
        v2 = target / np.linalg.norm(target)
        
        # Rotation axis is cross product
        axis = np.cross(v1, v2)
        axis_length = np.linalg.norm(axis)
        
        # Check if vectors are parallel or anti-parallel
        if axis_length < 1e-10:
            if np.dot(v1, v2) > 0:
                # Already aligned
                return Vector([1, 0, 0]), 0.0
            else:
                # Anti-parallel - rotate 180° around any perpendicular axis
                if abs(v1[0]) < 0.9:
                    axis = np.cross(v1, [1, 0, 0])
                else:
                    axis = np.cross(v1, [0, 1, 0])
                return Vector(axis / np.linalg.norm(axis)), 180.0
        
        axis = axis / axis_length
        
        # Angle is arccos of dot product
        angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
        
        return Vector(axis), np.degrees(angle)