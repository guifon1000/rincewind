"""
JSON-based Geometric Description Language (GDL) Parser
Simple, clean, and LLM-friendly format
"""

import json
from typing import Any, List, Dict, Optional, Union
from dataclasses import dataclass, asdict

# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class GDLPoint:
    x: float
    y: float
    z: float
    name: Optional[str] = None

    def __repr__(self):
        if self.name:
            return f"Point({self.name}: {self.x}, {self.y}, {self.z})"
        return f"Point({self.x}, {self.y}, {self.z})"


@dataclass
class GDLLine:
    start: int  # index in points array
    end: int
    name: Optional[str] = None

    def __repr__(self):
        if self.name:
            return f"Line({self.name}: {self.start} -> {self.end})"
        return f"Line({self.start} -> {self.end})"




@dataclass
class GDLPolyline:
    points: List[int]  # indices in points array
    name: Optional[str] = None

    def __repr__(self):
        if self.name:
            return f"Polyline({self.name}: {len(self.points)} points)"
        return f"Polyline({len(self.points)} points)"


@dataclass
class GDL_SrfPt:
    points: List[int]  # indices in points array
    name: Optional[str] = None

    def __repr__(self):
        if self.name:
            return f"SrfPt({self.name}: {len(self.points)} points)"
        return f"SrfPt({len(self.points)} points)"






# ============================================================================
# GEOMETRY CONTEXT
# ============================================================================

class GeometryContext:
    """Manages all geometry objects and resolves references"""

    def __init__(self):
        self.points = []
        self.lines = []
        self.polylines = []
        self.srf_pts = []

    def add_point(self, obj: GDLPoint):
        self.points.append(obj)

    def add_line(self, obj: GDLLine):
        self.lines.append(obj)

    def add_polyline(self, obj: GDLPolyline):
        self.polylines.append(obj)

    def add_srf_pt(self, obj: GDL_SrfPt):
        self.srf_pts.append(obj)


    def summary(self):
        """Print summary of all geometry"""
        print("=" * 60)
        print("GEOMETRY SUMMARY")
        print("=" * 60)
        print(f"Points:    {len(self.points)}")
        print(f"Lines:     {len(self.lines)}")
        print(f"Polylines: {len(self.polylines)}")
        print(f"SrfPts:    {len(self.srf_pts)}")
        print("=" * 60)

    def _dict_without_none_name(self, obj):
        """Convert dataclass to dict, excluding 'name' key if it's None"""
        d = asdict(obj)
        if 'name' in d and d['name'] is None:
            del d['name']
        return d

    def to_json(self):
        """Export entire context to JSON"""
        return {
            "points": [self._dict_without_none_name(p) for p in self.points],
            "lines": [self._dict_without_none_name(l) for l in self.lines],
            "polylines": [self._dict_without_none_name(p) for p in self.polylines],
            "srf_pts": [self._dict_without_none_name(s) for s in self.srf_pts],
        }


# ============================================================================
# JSON PARSER
# ============================================================================

class JSONGDLParser:
    """Parser for JSON-based Geometric Description Language"""

    def __init__(self):
        self.context = GeometryContext()

    def parse(self, json_data: Union[str, dict, list]):
        """Parse JSON GDL data

        Can accept:
        - JSON string
        - Python dict (single object or full document)
        - Python list (array of objects)
        """
        if isinstance(json_data, str):
            data = json.loads(json_data)
        else:
            data = json_data

        # Handle different input formats
        if isinstance(data, list):
            # Array of objects
            for obj in data:
                self._parse_object(obj)
        elif isinstance(data, dict):
            if 'objects' in data:
                # Document with objects array
                for obj in data['objects']:
                    self._parse_object(obj)
            elif any(key in data for key in ['points', 'lines', 'polylines', 'srf_pts']):
                # Exporter format with geometry arrays
                self._parse_geometry_dict(data)
            else:
                # Single object
                self._parse_object(data)

        return self.context

    def _parse_geometry_dict(self, data: dict):
        """Parse geometry dictionary with arrays (exporter format)"""
        # Parse points
        for point_data in data.get('points', []):
            point = GDLPoint(
                x=point_data['x'],
                y=point_data['y'],
                z=point_data['z'],
                name=point_data.get('name')
            )
            self.context.add_point(point)

        # Parse lines
        for line_data in data.get('lines', []):
            line = GDLLine(
                start=line_data['start'],
                end=line_data['end'],
                name=line_data.get('name')
            )
            self.context.add_line(line)

        # Parse polylines
        for polyline_data in data.get('polylines', []):
            polyline = GDLPolyline(
                points=polyline_data['points'],
                name=polyline_data.get('name')
            )
            self.context.add_polyline(polyline)

        # Parse surface points
        for srf_data in data.get('srf_pts', []):
            srf_pt = GDL_SrfPt(
                points=srf_data['points'],
                name=srf_data.get('name')
            )
            self.context.add_srf_pt(srf_pt)

    def _parse_object(self, obj: dict):
        """Parse a single object"""
        obj_type = obj.get('type')

        if obj_type == 'point':
            point = GDLPoint(
                x=obj['x'],
                y=obj['y'],
                z=obj['z'],
                name=obj.get('name')
            )
            self.context.add_point(point)

        elif obj_type == 'line':
            line = GDLLine(
                start=obj['start'],
                end=obj['end'],
                name=obj.get('name')
            )
            self.context.add_line(line)

        elif obj_type == 'polyline':
            polyline = GDLPolyline(
                points=obj['points'],
                name=obj.get('name')
            )
            self.context.add_polyline(polyline)

        elif obj_type == 'srf_pt':
            srf_pt = GDL_SrfPt(
                points=obj['points'],
                name=obj.get('name')
            )
            self.context.add_srf_pt(srf_pt)

        else:
            raise ValueError(f"Unknown object type: {obj_type}")

    def parse_file(self, filepath: str):
        """Parse JSON GDL from a file"""
        with open(filepath, 'r') as f:
            return self.parse(f.read())


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    # Example: Simple rectangle with lines
    rectangle_json = """
    [
        {"type": "point", "x": 0, "y": 0, "z": 0},
        {"type": "point", "x": 100, "y": 0, "z": 0},
        {"type": "point", "x": 100, "y": 50, "z": 0},
        {"type": "point", "x": 0, "y": 50, "z": 0},
        {"type": "line", "start": 0, "end": 1},
        {"type": "line", "start": 1, "end": 2},
        {"type": "line", "start": 2, "end": 3},
        {"type": "line", "start": 3, "end": 0},
        {"type": "polyline", "points": [0, 1, 2, 3, 0]},
        {"type": "srf_pt", "points": [0, 1, 2, 3]}
    ]
    """

    # Parse and display
    print("=" * 60)
    print("EXAMPLE: Simple Rectangle")
    print("=" * 60)
    parser = JSONGDLParser()
    ctx = parser.parse(rectangle_json)
    ctx.summary()
    print("\nPoints:")
    for i, pt in enumerate(ctx.points):
        print(f"  [{i}] {pt}")
    print("\nLines:")
    for i, line in enumerate(ctx.lines):
        print(f"  [{i}] {line}")
    print("\nPolylines:")
    for i, pl in enumerate(ctx.polylines):
        print(f"  [{i}] {pl}")
    print("\nSurface Points:")
    for i, srf in enumerate(ctx.srf_pts):
        print(f"  [{i}] {srf}")

    # Export back to JSON
    print("\n" + "=" * 60)
    print("EXPORT TO JSON")
    print("=" * 60)
    exported = ctx.to_json()
    print(json.dumps(exported, indent=2))