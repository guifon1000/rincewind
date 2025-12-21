"""
Wigley Hull Generator

This module provides functionality to create and visualize Wigley hull surfaces.
A Wigley hull is a mathematical ship hull model commonly used for hydrodynamic studies.
It's defined by a simple mathematical formula and is often used as a benchmark for
numerical methods in naval architecture.

The hull has a parabolic section shape and a parabolic waterline.
"""

import matplotlib.pyplot as plt
from rincewind.geometry.UVSurface import WigleySurface


def create_wigley_hull(length=40.0, width=10.0, draft=3.2):
    """
    Create a Wigley hull surface with the specified dimensions.
    
    Args:
        length (float): Length of the hull
        width (float): Maximum beam (width) of the hull
        draft (float): Draft (depth) of the hull
        
    Returns:
        WigleySurface: A parametric surface representing the Wigley hull
    """
    return WigleySurface(length, width, draft)


# Default hull dimensions
length = 40.0  # Hull length
width = 10.0   # Hull maximum beam (width)
draft = 3.2    # Hull draft (depth)

# Create a Wigley hull surface with the default dimensions
wigley = WigleySurface(length, width, draft)

# Visualization resolution parameters
Nx = 100  # Number of points along the length
Nz = 100  # Number of points along the draft

# Visualize the hull with curves of constant x-position (stations)
for i in range(Nx):
    curve_x = []
    # Parametric u-coordinate (0 to 1 along length)
    u = float(i) / float(Nx-1)
    
    for j in range(Nz):
        # Parametric v-coordinate (0 to 1 along draft)
        v = float(j) / float(Nz-1)
        
        # Get the 3D point at (u,v)
        p = wigley.get_point(u, v)
        
        # Store y,z coordinates for plotting
        curve_x.append([p[1], p[2]])
        
    # Plot the curve at this x-position
    plt.plot([p[0] for p in curve_x], [p[1] for p in curve_x])

# Ensure the aspect ratio of the plot is equal
plt.axis('equal')
plt.title('Wigley Hull - Cross Sections')
plt.xlabel('Y - Beam')
plt.ylabel('Z - Draft')
plt.show()

