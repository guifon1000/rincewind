"""Tests for the Rincewind to GDL exporter."""
import json, tempfile
from pathlib import Path
import pytest
from rincewind.geometry.Point import Point
from rincewind.geometry.Segment import Segment
from rincewind.geometry.Polyline3D import Polyline3D
from rincewind.document.exporter import RincewindExporter

class TestPointExport:
    def test_export_single_point(self):
        point = Point([1.0, 2.0, 3.0])
        exporter = RincewindExporter()
        ref = exporter.add_point(point, "origin")
        assert ref == 0
        assert len(exporter.context.points) == 1

class TestSegmentExport:
    def test_export_segment(self):
        p1 = Point([0.0, 0.0, 0.0])
        p2 = Point([1.0, 0.0, 0.0])
        segment = Segment([p1, p2])
        exporter = RincewindExporter()
        ref = exporter.add_segment(segment, "edge")
        assert ref == 0
        assert len(exporter.context.lines) == 1

class TestPolylineExport:
    def test_export_simple_polyline(self):
        points = [Point([0.0, 0.0, 0.0]), Point([1.0, 0.0, 0.0]), Point([1.0, 1.0, 0.0])]
        polyline = Polyline3D(points)
        exporter = RincewindExporter()
        ref = exporter.add_polyline(polyline, "triangle")
        assert ref == 0
        assert len(exporter.context.polylines) == 1
