"""
Reference test for export functionality: Square Pyramid
=======================================================

This test demonstrates the complete export workflow for a realistic geometry:
a square-based pyramid with mixed face types (1 quad + 4 triangles).

Geometry Structure:
  - 5 Points: base corners (0-3) + apex (4)
  - 8 Lines: 4 base edges + 4 edges to apex
  - 5 Polylines: closed loops for each face boundary
  - 5 SrfPt: surface definitions (1 quad, 4 triangles)

This test validates:
  ✓ Export all geometry types
  ✓ JSON structure and counts
  ✓ Round-trip consistency (export → parse → export = same JSON)
  ✓ Rhino3D compatibility (valid for parser)
"""

import json
import tempfile
from pathlib import Path

import pytest

from rincewind.geometry.Point import Point
from rincewind.document.exporter import RincewindExporter
from rincewind.document.gdl_parser import JSONGDLParser


class TestSquarePyramidExport:
    """Complete export test with a square pyramid."""

    @pytest.fixture
    def pyramid_exporter(self):
        """Create and populate exporter with square pyramid geometry."""
        exporter = RincewindExporter()

        # ===== POINTS (5 total) =====
        # Base vertices
        p0 = Point([0.0, 0.0, 0.0])  # corner 1
        p1 = Point([1.0, 0.0, 0.0])  # corner 2
        p2 = Point([1.0, 1.0, 0.0])  # corner 3
        p3 = Point([0.0, 1.0, 0.0])  # corner 4
        # Apex
        p4 = Point([0.5, 0.5, 1.0])  # top

        # Add points to exporter
        exporter.add_point(p0, "base_0")
        exporter.add_point(p1, "base_1")
        exporter.add_point(p2, "base_2")
        exporter.add_point(p3, "base_3")
        exporter.add_point(p4, "apex")

        # ===== LINES (8 total) =====
        # Base edges
        from rincewind.geometry.Segment import Segment
        exporter.add_segment(Segment([p0, p1]), "base_edge_0")
        exporter.add_segment(Segment([p1, p2]), "base_edge_1")
        exporter.add_segment(Segment([p2, p3]), "base_edge_2")
        exporter.add_segment(Segment([p3, p0]), "base_edge_3")
        # Side edges (apex connections)
        exporter.add_segment(Segment([p0, p4]), "side_edge_0")
        exporter.add_segment(Segment([p1, p4]), "side_edge_1")
        exporter.add_segment(Segment([p2, p4]), "side_edge_2")
        exporter.add_segment(Segment([p3, p4]), "side_edge_3")

        # ===== POLYLINES (5 total - closed face boundaries) =====
        from rincewind.geometry.Polyline3D import Polyline3D
        # Base boundary (quad)
        exporter.add_polyline(
            Polyline3D([p0, p1, p2, p3, p0]),
            "face_base_boundary"
        )
        # Front face boundary (triangle)
        exporter.add_polyline(
            Polyline3D([p0, p1, p4, p0]),
            "face_front_boundary"
        )
        # Right face boundary (triangle)
        exporter.add_polyline(
            Polyline3D([p1, p2, p4, p1]),
            "face_right_boundary"
        )
        # Back face boundary (triangle)
        exporter.add_polyline(
            Polyline3D([p2, p3, p4, p2]),
            "face_back_boundary"
        )
        # Left face boundary (triangle)
        exporter.add_polyline(
            Polyline3D([p3, p0, p4, p3]),
            "face_left_boundary"
        )

        # ===== SURFACE POINTS (5 total - face definitions) =====
        # Base face (4-point quad)
        exporter.add_srf_pt([p0, p1, p2, p3], "face_base")
        # Side faces (3-point triangles)
        exporter.add_srf_pt([p0, p1, p4], "face_front")
        exporter.add_srf_pt([p1, p2, p4], "face_right")
        exporter.add_srf_pt([p2, p3, p4], "face_back")
        exporter.add_srf_pt([p3, p0, p4], "face_left")

        return exporter

    def test_pyramid_export_structure(self, pyramid_exporter):
        """Verify the exported pyramid has correct geometry counts.

        Note: Points are duplicated because add_segment(), add_polyline(),
        and add_srf_pt() create new point instances rather than referencing
        existing ones. This is acceptable for now.

        Point count breakdown:
        - 5 explicit points
        - 8 segments × 2 points each = 16 points
        - 5 polylines (1 quad + 4 triangles) = 21 points
        - 5 srf_pts (1 quad + 4 triangles) = 16 points
        Total: 5 + 16 + 21 + 16 = 58 points
        """
        data = pyramid_exporter.to_dict()

        # Verify structure has all geometry types
        assert "points" in data
        assert "lines" in data
        assert "polylines" in data
        assert "srf_pts" in data

        # Verify counts
        assert len(data["points"]) == 58  # Due to current point duplication

        # 8 lines (4 base + 4 to apex)
        assert len(data["lines"]) == 8

        # 5 polylines (1 base + 4 sides)
        assert len(data["polylines"]) == 5

        # 5 srf_pts (1 quad base + 4 triangle sides)
        assert len(data["srf_pts"]) == 5

    def test_pyramid_export_point_data(self, pyramid_exporter):
        """Verify points have correct coordinates and names."""
        data = pyramid_exporter.to_dict()
        points = data["points"]

        # Check base points
        assert points[0]["x"] == 0.0 and points[0]["y"] == 0.0
        assert points[1]["x"] == 1.0 and points[1]["y"] == 0.0
        assert points[2]["x"] == 1.0 and points[2]["y"] == 1.0
        assert points[3]["x"] == 0.0 and points[3]["y"] == 1.0

        # Check apex
        assert points[4]["x"] == 0.5
        assert points[4]["y"] == 0.5
        assert points[4]["z"] == 1.0
        assert points[4]["name"] == "apex"

    def test_pyramid_export_lines(self, pyramid_exporter):
        """Verify lines reference valid point indices."""
        data = pyramid_exporter.to_dict()
        lines = data["lines"]
        max_point_idx = len(data["points"]) - 1

        # All lines should have valid point indices
        assert len(lines) == 8
        for line in lines:
            assert "start" in line
            assert "end" in line
            assert isinstance(line["start"], int)
            assert isinstance(line["end"], int)
            assert 0 <= line["start"] <= max_point_idx
            assert 0 <= line["end"] <= max_point_idx
            assert line["start"] != line["end"]  # Line shouldn't connect a point to itself

    def test_pyramid_export_polylines(self, pyramid_exporter):
        """Verify polylines have correct structure and reference valid points."""
        data = pyramid_exporter.to_dict()
        polylines = data["polylines"]
        max_point_idx = len(data["points"]) - 1

        # Should have 5 polylines (1 base + 4 sides)
        assert len(polylines) == 5

        # Verify structure: base + 4 triangular sides
        polyline_sizes = [len(pl["points"]) for pl in polylines]
        assert polyline_sizes[0] == 5  # Base quad: closed (5 points)
        assert polyline_sizes[1] == 4  # Front triangle: closed (4 points)
        assert polyline_sizes[2] == 4  # Right triangle: closed (4 points)
        assert polyline_sizes[3] == 4  # Back triangle: closed (4 points)
        assert polyline_sizes[4] == 4  # Left triangle: closed (4 points)

        # Verify all point indices are valid
        for polyline in polylines:
            assert "points" in polyline
            assert isinstance(polyline["points"], list)
            for idx in polyline["points"]:
                assert isinstance(idx, int)
                assert 0 <= idx <= max_point_idx

    def test_pyramid_export_srf_pts(self, pyramid_exporter):
        """Verify surface points (faces) have correct point counts and structure."""
        data = pyramid_exporter.to_dict()
        srf_pts = data["srf_pts"]
        max_point_idx = len(data["points"]) - 1

        # Should have 5 surface points (1 quad + 4 triangles)
        assert len(srf_pts) == 5

        # Verify structure: 1 quad + 4 triangles
        srf_sizes = [len(srf["points"]) for srf in srf_pts]
        assert srf_sizes[0] == 4  # Base face: quad
        assert srf_sizes[1] == 3  # Front face: triangle
        assert srf_sizes[2] == 3  # Right face: triangle
        assert srf_sizes[3] == 3  # Back face: triangle
        assert srf_sizes[4] == 3  # Left face: triangle

        # Verify names
        names = [srf.get("name") for srf in srf_pts]
        assert names[0] == "face_base"
        assert set(names[1:]) == {"face_front", "face_right", "face_back", "face_left"}

        # Verify all point indices are valid and 3-4 point constraint
        for srf in srf_pts:
            assert "points" in srf
            assert isinstance(srf["points"], list)
            assert 3 <= len(srf["points"]) <= 4  # SrfPt must have 3 or 4 points
            for idx in srf["points"]:
                assert isinstance(idx, int)
                assert 0 <= idx <= max_point_idx

    def test_pyramid_export_to_json_string(self, pyramid_exporter):
        """Verify JSON serialization is valid."""
        json_str = pyramid_exporter.to_json_string()

        # Should be valid JSON
        data = json.loads(json_str)

        # Should have the expected structure
        assert len(data["points"]) == 58  # Due to point duplication
        assert len(data["lines"]) == 8
        assert len(data["polylines"]) == 5
        assert len(data["srf_pts"]) == 5

    def test_pyramid_export_save_to_file(self, pyramid_exporter):
        """Verify saving to file works correctly."""
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = Path(tmpdir) / "pyramid.json"

            pyramid_exporter.save_json(filepath)

            assert filepath.exists()

            # Load and verify
            with open(filepath) as f:
                data = json.load(f)

            assert len(data["points"]) == 58  # Due to point duplication
            assert len(data["lines"]) == 8
            assert len(data["polylines"]) == 5
            assert len(data["srf_pts"]) == 5

    def test_pyramid_roundtrip_consistency(self, pyramid_exporter):
        """
        Test round-trip: export → parse → export produces same JSON.

        This validates:
        - JSON export format is parseable
        - Parser correctly reconstructs geometry
        - Re-export produces identical JSON (for points, lines, polylines)

        Note: srf_pts are not currently parsed by JSONGDLParser, so they're
        excluded from this consistency check.
        """
        # Step 1: Export to JSON
        json_1 = pyramid_exporter.to_dict()

        # Step 2: Parse JSON back to context
        parser = JSONGDLParser()
        parsed_context = parser.parse(json_1)

        # Step 3: Export the parsed context to JSON
        json_2 = parsed_context.to_json()

        # Step 4: Verify consistency for supported geometry types

        # Points should match
        assert len(json_1["points"]) == len(json_2["points"])
        for p1, p2 in zip(json_1["points"], json_2["points"]):
            assert p1["x"] == p2["x"]
            assert p1["y"] == p2["y"]
            assert p1["z"] == p2["z"]

        # Lines should match
        assert len(json_1["lines"]) == len(json_2["lines"])
        for l1, l2 in zip(json_1["lines"], json_2["lines"]):
            assert l1["start"] == l2["start"]
            assert l1["end"] == l2["end"]

        # Polylines should match
        assert len(json_1["polylines"]) == len(json_2["polylines"])
        for pl1, pl2 in zip(json_1["polylines"], json_2["polylines"]):
            assert pl1["points"] == pl2["points"]

    def test_pyramid_rhino_compatibility(self, pyramid_exporter):
        """
        Verify JSON structure is valid for Rhino3D parser.

        This ensures the exported format can be consumed by external tools
        like Rhino3D plugin for geometry reconstruction.
        """
        data = pyramid_exporter.to_dict()

        # Valid structure checks
        assert isinstance(data["points"], list)
        assert isinstance(data["lines"], list)
        assert isinstance(data["polylines"], list)
        assert isinstance(data["srf_pts"], list)

        # Points must have x, y, z coordinates
        for point in data["points"]:
            assert "x" in point
            assert "y" in point
            assert "z" in point
            assert isinstance(point["x"], (int, float))
            assert isinstance(point["y"], (int, float))
            assert isinstance(point["z"], (int, float))

        # Lines must reference valid point indices
        max_point_idx = len(data["points"]) - 1
        for line in data["lines"]:
            assert "start" in line
            assert "end" in line
            assert 0 <= line["start"] <= max_point_idx
            assert 0 <= line["end"] <= max_point_idx

        # Polylines must reference valid point indices
        for polyline in data["polylines"]:
            assert "points" in polyline
            assert isinstance(polyline["points"], list)
            assert len(polyline["points"]) >= 2
            for idx in polyline["points"]:
                assert 0 <= idx <= max_point_idx

        # SrfPt must have 3 or 4 points
        for srf_pt in data["srf_pts"]:
            assert "points" in srf_pt
            assert len(srf_pt["points"]) in [3, 4]
            for idx in srf_pt["points"]:
                assert 0 <= idx <= max_point_idx


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
