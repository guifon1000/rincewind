"""
Exporter for Rincewind geometry objects to GDL JSON format.
Converts Rincewind objects (Point, Polyline3D, Segment, etc.) to GDL JSON syntax.
"""

import json
from typing import Any, Dict, List, Optional, Union
from pathlib import Path

from rincewind.document.gdl_parser import GeometryContext, GDLPoint, GDLLine, GDLPolyline, GDL_SrfPt
from rincewind.geometry.Point import Point
from rincewind.geometry.Segment import Segment
from rincewind.geometry.Polyline3D import Polyline3D


class RincewindExporter:
    """Export Rincewind geometry objects to GDL JSON format."""

    def __init__(self):
        self.context = GeometryContext()

    def add_point(self, point: Point, name: Optional[str] = None) -> int:
        """
        Add a Rincewind Point to the context.

        Args:
            point: Rincewind Point object
            name: Optional name for the point

        Returns:
            Index of this point in the points array
        """
        if not isinstance(point, Point):
            raise TypeError(f"Expected Point, got {type(point)}")

        gdl_point = GDLPoint(
            x=float(point[0]),
            y=float(point[1]),
            z=float(point[2]),
            name=name
        )
        idx = len(self.context.points)
        self.context.add_point(gdl_point)
        return idx
    


    def add_segment(self, segment: Union[Segment, list, tuple], name: Optional[str] = None) -> int:
        """
        Add a Rincewind Segment (line) to the context.
        Creates points and a line connecting them.

        Args:
            segment: Rincewind Segment object or list/tuple with [start_point, end_point]
            name: Optional name for the segment

        Returns:
            Index of the line in the lines array
        """
        # Handle both Segment objects and list/tuple format
        if isinstance(segment, Segment):
            if len(segment) != 2:
                raise ValueError("Expected Segment with exactly 2 points")
            start_pt = segment[0]
            end_pt = segment[1]
        elif isinstance(segment, (list, tuple)) and len(segment) == 2:
            start_pt = segment[0]
            end_pt = segment[1]
        else:
            raise TypeError("Expected Segment or list/tuple with 2 points")

        # Add points and get their indices
        start_idx = self.add_point(start_pt)
        end_idx = self.add_point(end_pt)

        # Add line
        gdl_line = GDLLine(
            start=start_idx,
            end=end_idx,
            name=name
        )
        line_idx = len(self.context.lines)
        self.context.add_line(gdl_line)
        return line_idx

    def add_polyline(self, polyline: Polyline3D, name: Optional[str] = None) -> int:
        """
        Add a Rincewind Polyline3D to the context.
        Creates points and a polyline connecting them.

        Args:
            polyline: Rincewind Polyline3D object
            name: Optional name for the polyline

        Returns:
            Index of the polyline in the polylines array
        """
        if not isinstance(polyline, Polyline3D):
            raise TypeError(f"Expected Polyline3D, got {type(polyline)}")

        if len(polyline) < 2:
            raise ValueError("Polyline must have at least 2 points")

        # Add all points and collect their indices
        point_indices = []
        for point in polyline:
            pt_idx = self.add_point(point)
            point_indices.append(pt_idx)

        # Add polyline
        gdl_polyline = GDLPolyline(
            points=point_indices,
            name=name
        )
        polyline_idx = len(self.context.polylines)
        self.context.add_polyline(gdl_polyline)
        return polyline_idx
    

    def add_srf_pt(self, pts: List[Point], name: Optional[str] = None) -> int:
        """
        Add a list of Rincewind Points as surface points to the context.
        
        Args:
            pts: List of Rincewind Point objects
            name: Optional name for the surface point group
        Returns:
            Index of this surface point group in the srf_pts array
        """
        if len(pts) < 2 or len(pts) > 4:
            raise ValueError("SrfPt must have at least 2 points and up to 4 points")
        
        # Add all points and collect their indices
        point_indices = []
        for point in pts:
            pt_idx = self.add_point(point)
            point_indices.append(pt_idx)

        # Add surface points
        gdl_srfpt = GDL_SrfPt(
            points=point_indices,
            name=name
        )
        srf_pt_idx = len(self.context.srf_pts)
        self.context.add_srf_pt(gdl_srfpt)
        return srf_pt_idx

    def to_dict(self) -> Dict[str, Any]:
        """Export entire context as a dictionary."""
        return self.context.to_json()

    def to_json_string(self, indent: int = 2) -> str:
        """Export entire context as a JSON string."""
        return json.dumps(self.to_dict(), indent=indent)

    def save_json(self, filepath: Union[str, Path]) -> None:
        """Save exported geometry to a JSON file."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            f.write(self.to_json_string())

    def reset(self) -> None:
        """Clear all data and start fresh."""
        self.context = GeometryContext()
        self._name_counter = {}

    def summary(self) -> str:
        """Get a summary of exported geometry."""
        self.context.summary()
