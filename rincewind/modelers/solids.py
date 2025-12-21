from rincewind.geometry.Point import Point
from typing import List, Tuple, Optional, Dict, Union
import math
from rincewind.modelers.rhomboids import RhomboidShape

# Test mesh creation functions
def create_test_tetrahedron() -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Create a simple tetrahedron for testing.
    
    This creates a basic triangular mesh that can be used to test
    the bar generation system.
    """
    vertices = [
        Point([0, 0, 0]),          # Bottom center
        Point([100, 0, 0]),        # Bottom right
        Point([50, 86.6, 0]),      # Bottom left (equilateral triangle)
        Point([50, 28.9, 81.6])    # Top apex
    ]
    
    faces = [
        (0, 1, 2),  # Bottom face
        (0, 1, 3),  # Front face
        (1, 2, 3),  # Right face  
        (2, 0, 3)   # Left face
    ]
    
    return vertices, faces


def create_test_cube() -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Create a triangulated cube for testing.
    
    Each face of the cube is split into 2 triangles, creating
    a mesh with 12 triangular faces.
    """
    vertices = [
        Point([0, 0, 0]),      # 0: bottom-front-left
        Point([100, 0, 0]),    # 1: bottom-front-right
        Point([100, 100, 0]),  # 2: bottom-back-right
        Point([0, 100, 0]),    # 3: bottom-back-left
        Point([0, 0, 100]),    # 4: top-front-left
        Point([100, 0, 100]),  # 5: top-front-right
        Point([100, 100, 100]),# 6: top-back-right
        Point([0, 100, 100])   # 7: top-back-left
    ]
    
    # Each face split into 2 triangles
    faces = [
        # Bottom face
        (0, 1, 2), (0, 2, 3),
        # Top face  
        (4, 6, 5), (4, 7, 6),
        # Front face
        (0, 5, 1), (0, 4, 5),
        # Back face
        (2, 7, 3), (2, 6, 7), 
        # Left face
        (0, 3, 7), (0, 7, 4),
        # Right face  
        (1, 6, 2), (1, 5, 6)
    ]
    
    return vertices, faces


def create_test_icosahedron() -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Create a geodesic icosahedron for advanced testing.
    
    This creates a more complex mesh with 20 triangular faces,
    useful for testing the system with multiple edge connections.
    """
    # Golden ratio
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    
    # Icosahedron vertices
    vertices = [
        # Rectangle in xy plane
        Point([-1, phi, 0]), Point([1, phi, 0]),
        Point([-1, -phi, 0]), Point([1, -phi, 0]),

        # Rectangle in xz plane
        Point([0, -1, phi]), Point([0, 1, phi]),
        Point([0, -1, -phi]), Point([0, 1, -phi]),

        # Rectangle in yz plane
        Point([phi, 0, -1]), Point([phi, 0, 1]),
        Point([-phi, 0, -1]), Point([-phi, 0, 1])
    ]

    # Scale up for reasonable dimensions (100mm)
    scale = 5
    vertices = [Point([v[0] * scale, v[1] * scale, v[2] * scale]) for v in vertices]
    
    # Icosahedron faces (20 triangles)
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
    ]
    
    return vertices, faces

def create_refined_icosahedron(subdivisions: int = 1, radius: float = 50.0) -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Create a refined icosahedron with vertices projected onto a sphere.
    
    Args:
        subdivisions: Number of subdivision iterations (0 = base icosahedron, 1 = 80 faces, 2 = 320 faces, etc.)
        radius: Radius of the sphere in mm
    
    Returns:
        vertices: List of vertices on the sphere surface
        faces: List of triangular faces
    """
    # Golden ratio
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    
    # Base icosahedron vertices (normalized)
    norm = math.sqrt(1 + phi * phi)
    vertices = [
        Point([-1/norm, phi/norm, 0]), Point([1/norm, phi/norm, 0]),
        Point([-1/norm, -phi/norm, 0]), Point([1/norm, -phi/norm, 0]),
        Point([0, -1/norm, phi/norm]), Point([0, 1/norm, phi/norm]),
        Point([0, -1/norm, -phi/norm]), Point([0, 1/norm, -phi/norm]),
        Point([phi/norm, 0, -1/norm]), Point([phi/norm, 0, 1/norm]),
        Point([-phi/norm, 0, -1/norm]), Point([-phi/norm, 0, 1/norm])
    ]
    
    # Base icosahedron faces
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
    ]
    
    # Subdivide the mesh
    for _ in range(subdivisions):
        vertices, faces = _subdivide_mesh(vertices, faces)
    
    # Scale to desired radius
    vertices = [Point([v[0] * radius, v[1] * radius, v[2] * radius]) for v in vertices]
    
    return vertices, faces


def _subdivide_mesh(vertices: List[Point], faces: List[Tuple[int, int, int]]) -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Subdivide each triangle into 4 smaller triangles and project new vertices onto unit sphere.
    
    Each triangle (a, b, c) becomes:
         a
        / 
       ab-ac
      / \ / 
     b--bc---c
    """
    new_vertices = list(vertices)
    new_faces = []
    midpoint_cache = {}  # Cache to avoid duplicate vertices
    
    def get_midpoint(i1: int, i2: int) -> int:
        """Get or create midpoint between two vertices, normalized to unit sphere."""
        # Use ordered tuple as key
        key = tuple(sorted([i1, i2]))
        
        if key in midpoint_cache:
            return midpoint_cache[key]
        
        # Calculate midpoint
        v1, v2 = vertices[i1], vertices[i2]
        mid = Point([
            (v1[0] + v2[0]) / 2.0,
            (v1[1] + v2[1]) / 2.0,
            (v1[2] + v2[2]) / 2.0
        ])

        # Normalize to unit sphere
        length = math.sqrt(mid[0]**2 + mid[1]**2 + mid[2]**2)
        mid = Point([mid[0] / length, mid[1] / length, mid[2] / length])
        
        # Add to vertices list
        idx = len(new_vertices)
        new_vertices.append(mid)
        midpoint_cache[key] = idx
        
        return idx
    
    # Subdivide each face
    for a, b, c in faces:
        # Get midpoints
        ab = get_midpoint(a, b)
        bc = get_midpoint(b, c)
        ac = get_midpoint(a, c)
        
        # Create 4 new faces
        new_faces.append((a, ab, ac))
        new_faces.append((b, bc, ab))
        new_faces.append((c, ac, bc))
        new_faces.append((ab, bc, ac))
    
    return new_vertices, new_faces

def create_triangulated_zome(radius, N) -> Tuple[List[Point], List[Tuple[int, int, int]]]:
    """
    Create a simple Zometool structure for testing.
    
    This creates a basic mesh that mimics a Zometool structure,
    useful for testing the bar generation system.
    """
    shape = RhomboidShape(radius, N, True)
    shape.create_mesh()
    print (shape.mesh.cells_dict["triangle"])
    face_vec = [(f[2], f[1], f[0]) for f in shape.mesh.cells_dict["triangle"]]
    return [Point([p[0], p[1], p[2]]) for p in shape.mesh.points], face_vec

def create_rhomboid_zome(radius, N) -> Tuple[List[Point], List[Tuple[int, int, int, int]]]:
    """
    Create a rhomboid Zometool structure for testing.
    
    This creates a basic mesh that mimics a Zometool structure,
    useful for testing the bar generation system.
    """
    shape = RhomboidShape(radius, N, False)
    shape.create_mesh()
    print (shape.mesh.cells_dict["quad"])
    face_vec = [f for f in shape.mesh.cells_dict["quad"]]
    print(shape.mesh.points)
    return [Point([p[0], p[1], p[2]]) for p in shape.mesh.points], face_vec