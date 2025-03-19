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
    
    def add_to_ax(self, ax, pt, scale=1.):
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
            length=1., normalize=False
        )

