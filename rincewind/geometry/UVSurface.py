"""
UV Surface Module

This module provides classes for parametric surfaces defined by UV coordinates.
These surfaces are fundamental for creating complex 3D objects in CAD and engineering applications.
"""

from Point import Point

class UVSurface(object):
    """
    Base class for parametric surfaces defined by UV coordinates.
    
    A UVSurface represents a surface in 3D space that is parameterized by two variables
    u and v, typically ranging from 0 to 1. Points on the surface are obtained by
    evaluating a bivariate function at specific (u,v) coordinates.
    """
    def __init__(self):
        """
        Initialize a UVSurface.
        
        Sets default identity mappings for u and v parameters and a null
        bivariate function that returns the origin point.
        """
        # Default parameter mappings (identity functions)
        self.fu = lambda s: s 
        self.fv = lambda s: s 

        # Default bivariate function returns the origin
        def null_uv(u, v):
            return Point([0., 0., 0.])
        self.bivariate_function = null_uv

    def set_bivariate_function(self, func):
        """
        Set the bivariate function that defines this surface.
        
        Args:
            func: A function taking parameters (u,v) and returning a Point
        """
        self.bivariate_function = func

    def get_point(self, u, v):
        """
        Get the 3D point at the specified UV coordinates.
        
        Args:
            u (float): First parameter value, typically in [0,1]
            v (float): Second parameter value, typically in [0,1]
            
        Returns:
            Point: The 3D point on the surface at (u,v)
        """
        return self.bivariate_function(u, v)

class WigleySurface(UVSurface):
    """
    A mathematical model of a Wigley hull surface.
    
    The Wigley hull is a standard benchmark hull form in naval architecture,
    defined by a simple mathematical formula. It has a parabolic section shape
    and a parabolic waterline.
    """
    def __init__(self, length, width, draft):
        """
        Initialize a Wigley hull surface with the specified dimensions.
        
        Args:
            length (float): Hull length
            width (float): Maximum beam (width) of the hull
            draft (float): Draft (depth) of the hull
        """
        self.length = length
        self.width = width
        self.draft = draft
        super(WigleySurface, self).__init__()
        
        # Define the Wigley hull mathematical function
        def bivariate(u, v):
            # Calculate coordinates
            x1 = -0.5 * length + u * length  # Intermediate x value
            x = -u * length                  # Final x value
            z1 = 0.5 * (-draft + v * draft)  # Intermediate z value
            z = (-draft + 2.* v * draft)     # Final z value
            
            # Calculate y using the Wigley hull formula
            y = 0.5 * width * (1. - (2. * x1 / length)**2.) * \
                    (1. - (2. * z1 / draft)**2.)
                    
            return Point([x, y, z])
            
        self.set_bivariate_function(bivariate)

    def sections(self, s, N, ax=None):
        """
        Generate a cross-section of the hull at a specified position.
        
        Creates a complete cross-section of the hull at the specified longitudinal
        position, including both port and starboard sides and the deck.
        
        Args:
            s (float): Longitudinal position parameter in [0,1]
                       (0 = stern, 1 = bow)
            N (int): Number of points to generate along each side of the hull
            ax (matplotlib.axes, optional): If provided, the section will be plotted
            
        Returns:
            Named3DSection: A named section containing hull and deck polylines
        """
        # Define functions for port and starboard sides
        fsurf_port = lambda z: self.bivariate_function(s, z)
        # Starboard side mirrors the port side in y-direction
        fsurf_starboard = lambda z: Point([
            self.bivariate_function(s, z)[0],
            -self.bivariate_function(s, z)[1],
            self.bivariate_function(s, z)[2]
        ])
        
        from rincewind.geometry.Named3DSection import Named3DSection
        
        # Define section component names
        lkeys = ['hull1', 'deck', 'hull2']  # Port, deck, starboard
        
        # Initialize collections for the points
        list_hull1 = []  # Port side
        list_deck = []   # Deck (top)
        list_hull2 = []  # Starboard side
        
        # Generate points along the hull sections
        for i in range(N+1):
            v1 = float(i)/float(N)  # Parameter for port side
            v2 = 1. - v1            # Parameter for starboard side (reversed)
            u = s                   # Longitudinal position is fixed
            
            # Add points to the collections
            list_hull1.append(fsurf_port(v1))
            list_hull2.append(fsurf_starboard(v2))
            
        # Connect the hull sides with a deck line
        list_deck = [list_hull1[-1], list_hull2[0]]
        
        # Combine all polylines
        list_polylines = [list_hull1, list_deck, list_hull2]
        
        # Create a named section from the polylines
        ns = Named3DSection(list_polylines, lkeys)
        
        # Visualize if requested
        if ax: 
            ns.whole_polyline.add_to_ax(ax)
            
        return ns

