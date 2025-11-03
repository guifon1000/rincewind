import numpy as np
from typing import List, Tuple, Dict
from .Point import Point
from .Vector import Vector
from .Plane import Plane
from .Line import Line
from .Segment import Segment
from ..geo_functions import cross, intersect_line_and_plane, is_on_segment, distance, is_on_plane
from .Polyline3D import Polyline3D
import matplotlib.pyplot as plt

class Bar:
    """
    Represents a wooden bar/beam with rectangular cross-section.
    
    This replaces the original Bar class which used extensive Rhino functions:
    - rs.coerce3dpoint() -> Point3D class
    - rs.VectorUnitize() -> normalize_vector()
    - rs.PlaneFromNormal() -> GeometryUtils.plane_from_normal()
    - rs.AddPolyline() -> direct vertex generation
    - rs.ExtrudeSurface() -> manual geometry creation
    - rs.SurfaceAreaCentroid() -> calculated center
    
    A bar is defined by:
    - Two end points (pt1, pt2) 
    - Cross-sectional dimensions (width x height) - the ep and rt parameters
    - Local coordinate system (direction, aplomb, side vectors)
    - Cutting planes for joints
    - Optional lengthwise splitting capability
    """
    
    def __init__(self, pt1: Point, pt2: Point, aplomb: np.ndarray = None, 
                 section: Tuple[float, float] = (0.15, 0.3), name: str = 'default'):
        """
        Initialize a bar between two points.
        
        Args:
            pt1, pt2: End points of the bar
            aplomb: Up direction vector for the bar orientation (replaces original aplomb param)
            section: Cross-section dimensions (ep, rt) - width and height in mm
            name: Identifier for the bar
        """
        self.pt1 = pt1
        self.pt2 = pt2
        self.section = section  # (ep=width, rt=height) from original code
        self.name = name
        self.cuts = []  # List of cutting planes
        self.length = distance(pt1, pt2)
        self.ep = section[0]
        self.rt = section[1]
        
        # Calculate bar direction vector
        # Replaces: self.directionVec = rs.VectorUnitize(rg.Vector3d(pt2[0]-pt1[0], pt2[1]-pt1[1], pt2[2]-pt1[2]))
        #self.direction_vec = normalize_vector(pt2.to_array() - pt1.to_array())

        dir_vec = Vector(pt2 - pt1)
        self.direction_vec = dir_vec.unit()

        # Set up local coordinate system
        # This replaces the complex aplomb vector calculation in the original
        self._setup_coordinate_system(aplomb)
        
        # Calculate key points
        # Replaces: self.mid = rg.Point3d(0.5*(pt1[0]+pt2[0]), 0.5*(pt1[1]+pt2[1]), 0.5*(pt1[2]+pt2[2]))
        self.midpoint = Point([
            0.5 * (pt1[0] + pt2[0]),
            0.5 * (pt1[1] + pt2[1]),
            0.5 * (pt1[2] + pt2[2])])
        # Generate bar geometry
        # Replaces: rs.ExtrudeSurface(), rs.AddPolyline(), etc.
        self._generate_geometry()
        # this gives me the 8 corner points of the bar at start and end
        self.start_corners = self._get_cross_section_points(self.pt1, self.ep, self.rt)
        self.end_corners = self._get_cross_section_points(self.pt2, self.ep, self.rt)
        self.colors = plt.cm.tab10.colors[:6]  # or plt.cm.Set3.colors[:6]

    def add_to_ax(self, ax, color='brown', alpha=0.8):
        for iface in range(6):
            poly = np.array(self.get_face_polyline(iface))
            self.get_face_polyline(iface).add_to_ax(ax, color='black')

    def _setup_coordinate_system(self, aplomb: np.ndarray = None):
        """
        Set up the local coordinate system for the bar.
        
        This replaces the original complex vector calculation:
        - Original used rs.VectorLength(rs.VectorCrossProduct(...)) for parallel checks
        - Original used rs.VectorCrossProduct() for perpendicular vectors
        - Original had special cases for vertical bars
        
        The bar has three perpendicular vectors:
        - direction_vec: along the bar length
        - aplomb_vec: perpendicular to bar, typically "up" (original aplombVec)
        - side_vec: perpendicular to both, forming right-handed system (original epVec)
        """
        if aplomb is None:
            aplomb = np.array([0, 0, -1])  # Default up direction
        
        # Handle special case when aplomb is parallel to direction
        # Replaces original: if (rs.VectorLength(rs.VectorCrossProduct((0,0,-1.),aplomb))<0.0001):
        if np.linalg.norm(cross(aplomb, self.direction_vec)) < 0.0001:
            if np.linalg.norm(cross(np.array([0, 0, -1]), self.direction_vec)) < 0.0001:
                # Direction is vertical, use X axis as reference
                # Original: left = [-1,0,0]
                side = np.array([-1, 0, 0])
                self.aplomb_vec = cross(side, self.direction_vec).unit()
            else:
                side = cross(self.direction_vec, aplomb).unit()
                self.aplomb_vec = cross(side, self.direction_vec).unit()
        else:
            side = cross(self.direction_vec, aplomb).unit()
            self.aplomb_vec = cross(side, self.direction_vec).unit()

        # Original: self.epVec = rs.VectorCrossProduct(self.aplombVec, self.directionVec)
        self.side_vec = cross(self.aplomb_vec, self.direction_vec).unit()
    
    def _generate_geometry(self):
        """
        Generate the 3D geometry of the bar with rectangular cross-section.
        
        This replaces the original geometry creation:
        - rs.AddPolyline([pts[i] for i in range(len(pts))]) 
        - rs.ExtrudeSurface(rs.AddSrfPt(pts[:-1]), rs.AddLine(pt1,pt2))
        - Complex point calculations with section[0] and section[1]
        """
        ep, rt = self.section  # ep=width, rt=height from original

        pt1_further = self.pt1 -  0.3 * Vector(self.pt2 - self.pt1) 
        pt2_further = self.pt2 + 0.3 * Vector(self.pt2 - self.pt1)

        # Create cross-section at start point
        start_corners = self._get_cross_section_points(pt1_further, ep, rt)
        end_corners = self._get_cross_section_points(pt2_further, ep, rt)

        midpoint_corners = self._get_cross_section_points(self.midpoint, ep, rt)

        self.start_corners = start_corners
        self.end_corners = end_corners
        self.midpoint_corners = midpoint_corners

        # Store geometry data for export/visualization
        self.geometry_data = {
            'start_corners': [p.to_array() for p in start_corners],
            'end_corners': [p.to_array() for p in end_corners],
            'faces': self._generate_faces()
        }
        
        self.top_plane = Plane((start_corners[0], end_corners[0], start_corners[3]))
        self.bottom_plane = Plane((start_corners[1], end_corners[1], start_corners[2]))
        self.side1_plane = Plane((start_corners[0], end_corners[0], start_corners[1]))
        self.side2_plane = Plane((start_corners[2], end_corners[2], start_corners[3]))
        # Calculate center of geometry for cutting operations
        # Replaces: self.cog = rs.SurfaceAreaCentroid(self.vpb)
        all_points = [p.to_array() for p in start_corners + end_corners]
        self.center_of_geometry = np.mean(all_points, axis=0)
    
    def _get_cross_section_points(self, center: Point, ep: float, rt: float) -> List[Point]:
        """
        Generate the 4 corner points of rectangular cross-section at given center.
        
        This replaces the original point creation:
        Original created pts[] with complex vector math:
        - pts.append(rg.Point3d(pt1[0], pt1[1], pt1[2]))
        - pts.append(rg.Point3d(pt1[0]+section[1]*self.aplombVec[0], ...))
        - pts.append(rg.Point3d(pt1[0]+section[1]*self.aplombVec[0]-section[0]*self.epVec[0], ...))
        - etc.
        
        Args:
            center: Center point of cross-section
            ep: Width of rectangle (along side_vec direction) - original section[0]
            rt: Height of rectangle (along aplomb_vec direction) - original section[1]
            
        Returns:
            List of 4 corner points in order matching original
        """
        center_array = center.to_array()
        
        # Generate corners exactly as in original code
        # Original order was: base, +aplomb, +aplomb-ep, -ep
        corners = []
        corners.append(center_array)  # Base point
        corners.append(center_array + rt * self.aplomb_vec)  # +aplomb (up)
        corners.append(center_array + rt * self.aplomb_vec + ep * self.side_vec)  # +aplomb-ep  
        corners.append(center_array + ep * self.side_vec)  # -ep (side)
        
        return [Point([p[0], p[1], p[2]]) for p in corners]
    
    def _generate_faces(self) -> List[List[int]]:
        """
        Generate face connectivity for the bar (rectangular prism).
        
        Replaces the original surface creation with explicit mesh topology.
        Original used: rs.ExtrudeSurface() which created NURBS surfaces
        """
        # 8 vertices total: 4 at start, 4 at end
        # Faces connect corresponding vertices
        faces = [
            [0, 1, 5, 4],  # bottom face
            [1, 2, 6, 5],  # left face  
            [2, 3, 7, 6],  # top face
            [3, 0, 4, 7],  # right face
            [0, 3, 2, 1],  # start cap
            [4, 5, 6, 7]   # end cap
        ]
        return faces
    
    def add_cutting_plane(self, plane: Plane):
        """
        Add a cutting plane that will modify this bar's geometry.
        
        Replaces original: setCut(planesrf) method which used:
        - rs.IntersectBreps() to find intersection curves
        - rs.SplitBrep() to split the bar geometry  
        - rs.CapPlanarHoles() to close the geometry
        - rg.Brep.IsPointInside() to find the correct piece
        """
        self.cuts.append(plane)

    def cut_by_plane(self, plane: Plane):
        """
        Cut the bar by a given plane.
        
        This is a simplified version of the original setCut() method.
        Actual geometry modification is not implemented here, just records the cut.
        
        Args:
            plane: Plane to cut the bar with
        """
        print(plane)
        start_points = self.start_corners
        end_points = self.end_corners
        line0 = Line((start_points[0], end_points[0]))
        line1 = Line((start_points[1], end_points[1]))
        line2 = Line((start_points[2], end_points[2]))
        line3 = Line((start_points[3], end_points[3]))
        inter0 = Point(intersect_line_and_plane(line0, plane))
        inter1 = Point(intersect_line_and_plane(line1, plane))
        inter2 = Point(intersect_line_and_plane(line2, plane))
        inter3 = Point(intersect_line_and_plane(line3, plane))
        if is_on_segment(self.midpoint_corners[0], Segment((Point(self.start_corners[0]), inter0))):
            self.end_corners[0] = inter0
        elif is_on_segment(self.midpoint_corners[0], Segment((inter0, Point(self.end_corners[0])))):
            self.start_corners[0] = inter0

        if is_on_segment(self.midpoint_corners[1], Segment((Point(self.start_corners[1]), inter1))):
            self.end_corners[1] = inter1
        elif is_on_segment(self.midpoint_corners[1], Segment((inter1, Point(self.end_corners[1])))):
            self.start_corners[1] = inter1

        if is_on_segment(self.midpoint_corners[2], Segment((Point(self.start_corners[2]), inter2))):
            self.end_corners[2] = inter2

        elif is_on_segment(self.midpoint_corners[2], Segment((inter2, Point(self.end_corners[2])))):
            self.start_corners[2] = inter2


        if is_on_segment(self.midpoint_corners[3], Segment((Point(self.start_corners[3]), inter3))):
            self.end_corners[3] = inter3
        elif is_on_segment(self.midpoint_corners[3], Segment((inter3, Point(self.end_corners[3])))):
            self.start_corners[3] = inter3


        all_intersections = [inter0, inter1, inter2, inter3]
        points = [p for p in all_intersections if p is not None]
        return points

    def get_face_polyline(self, face_index: int) -> List[Point]:
        """
        Get the polyline points for a given face of the bar.
        
        Args:
            face_index: Index of the face (0-5)
        Returns:
            List[Point]: List of points defining the polyline for the face
        """
        return Polyline3D([Point(self.get_vertices()[i]) for i in self._generate_faces()[face_index]])

    def add_lengthwise_split(self, plane: Plane):
        """
        Add a lengthwise splitting plane perpendicular to the bar.
        
        This is new functionality - the original code didn't have lengthwise splits,
        but you mentioned this as a desired feature.
        
        Args:
            plane: Plane to split the bar along its length
        """
        # pt1 distance to plane
        if is_on_plane(self.pt1, plane) and is_on_plane(self.pt2, plane):
            print('Bar already lies on the splitting plane. Fuck you')
            
        
        line_start = Line((self.start_corners[2], self.start_corners[3]))
        self.start_corners[3] = intersect_line_and_plane(line_start, plane)

        line_end = Line((self.end_corners[2], self.end_corners[3]))
        self.end_corners[3] = intersect_line_and_plane(line_end, plane)

    def get_vertices(self) -> List[np.ndarray]:
        """
        Get all vertices of the bar as numpy arrays.
        
        Replaces original complex point management with simple vertex list.
        """
        vertices = []
        vertices.extend([p for p in self.start_corners])
        vertices.extend([p for p in self.end_corners])
        return vertices
    
    def export_data(self) -> Dict:
        """
        Export bar data for external processing.
        
        This replaces the original orient1(), orient2(), orient3() methods
        which were for positioning bars in Rhino. Instead, we export raw data.
        """
        return {
            'name': self.name,
            'start_point': self.pt1.to_array().tolist(),
            'end_point': self.pt2.to_array().tolist(),
            'section': self.section,  # (ep, rt)
            'length': compute_distance(self.pt1, self.pt2),
            'direction': self.direction_vec.tolist(),
            'aplomb': self.aplomb_vec.tolist(),
            'side': self.side_vec.tolist(),
            'vertices': [v.tolist() for v in self.get_vertices()],
            'faces': self._generate_faces(),
            'num_cuts': len(self.cuts)
        }

class BarPolygon:
    def __init__(self, center: Point, points: List[Point]):
        self.points = points
        self.edges = [(i, (i + 1) % len(points)) for i in range(len(points))]
        self.faces = [tuple(range(len(points)))]
        self.center = center
        self.bars = []
        for e in self.edges:
            mid = Point([0.5 * (points[e[0]][i] + points[e[1]][i]) for i in range(3)])
            apb0 = Vector(center - mid).unit()
            bar = Bar(points[e[0]], points[e[1]], aplomb=apb0)
            self.bars.append(bar)
        print('Created BarPolygon with center at', center)
    
    def is_planar(self) -> bool:
        # Simple planarity check for polygon
        if len(self.points) < 4:
            return True  # Triangles are always planar
        p1, p2, p3 = self.points[0], self.points[1], self.points[2]
        normal = cross(Vector(p2 - p1), Vector(p3 - p1))
        for p in self.points[3:]:
            v = Vector(p - p1)
            if abs(normal.dot(v)) > 1e-6:  # Tolerance for planarity
                return False
        return True

    def get_edges(self) -> List[Tuple[int, int]]:
        return self.edges

    def get_faces(self) -> List[Tuple[int, ...]]:
        return self.faces
    
    def create_end_cuts(self):
        for ibar,bar in enumerate(self.bars):
            next = self.bars[ibar + 1] if ibar + 1 < len(self.bars) else self.bars[0]
            prev = self.bars[ibar - 1] if ibar - 1 >= 0 else self.bars[-1]
            bar.add_cutting_plane(next.side1_plane)
            bar.add_cutting_plane(prev.side2_plane)
            bar.cut_by_plane(next.side1_plane)
            bar.cut_by_plane(prev.side2_plane)


class BarQuad:
    def __init__(self, center: Point, p1: Point, p2: Point, p3: Point, p4: Point):
        self.points = [p1, p2, p3, p4]
        self.edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        self.faces = [(0, 1, 2, 3)]
        self.center = center
        print('Created BarQuad with center at', center)
    
    def is_planar(self) -> bool:
        # Simple planarity check for quadrilateral
        p1, p2, p3, p4 = self.points
        v1 = Vector([p2[i] - p1[i] for i in range(3)])  # p2 - p1
        v2 = Vector([p3[i] - p1[i] for i in range(3)])  # p3 - p1
        v3 = Vector([p4[i] - p1[i] for i in range(3)])  # p4 - p1
        normal = cross(v1, v2)
        return abs(normal.dot(v3)) < 1e-6  # Tolerance for planarity

    def get_edges(self) -> List[Tuple[int, int]]:
        return self.edges

    def get_faces(self) -> List[Tuple[int, int, int, int]]:
        return self.faces
