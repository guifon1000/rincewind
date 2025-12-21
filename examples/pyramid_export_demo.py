"""
Example: Square Pyramid Export

Demonstrates exporting a complete 3D geometry (square-based pyramid)
with mixed face types (1 quad base + 4 triangular sides).

This example shows:
- Creating points
- Adding surface definitions (srf_pt)
- Exporting to JSON
- Saving to file
"""

import json
from rincewind.geometry.Point import Point
from rincewind.document.exporter import RincewindExporter


def main():
    exporter = RincewindExporter()

    # Create pyramid points
    p0 = Point([0.0, 0.0, 0.0])  # base corner 1
    p1 = Point([1.0, 0.0, 0.0])  # base corner 2
    p2 = Point([1.0, 1.0, 0.0])  # base corner 3
    p3 = Point([0.0, 1.0, 0.0])  # base corner 4
    p4 = Point([0.5, 0.5, 1.0])  # apex

    # Add points to exporter
    exporter.add_point(p0, "base_0")
    exporter.add_point(p1, "base_1")
    exporter.add_point(p2, "base_2")
    exporter.add_point(p3, "base_3")
    exporter.add_point(p4, "apex")

    # Add surface definitions (faces)
    # Base face: 4-point quad
    exporter.add_srf_pt([p0, p1, p2, p3], "face_base")

    # Side faces: 3-point triangles
    exporter.add_srf_pt([p0, p1, p4], "face_front")
    exporter.add_srf_pt([p1, p2, p4], "face_right")
    exporter.add_srf_pt([p2, p3, p4], "face_back")
    exporter.add_srf_pt([p3, p0, p4], "face_left")

    # Save to file
    exporter.save_json("pyramid_output.json")

    # Print summary
    data = json.loads(exporter.to_json_string())
    print("✓ Pyramid exported successfully")
    print(f"  Points: {len(data['points'])}")
    print(f"  Lines: {len(data['lines'])}")
    print(f"  Polylines: {len(data['polylines'])}")
    print(f"  Surface Points: {len(data['srf_pts'])}")
    print()
    print("First 3 points:")
    for i, p in enumerate(data["points"][:3]):
        print(f"  [{i}] {p}")
    print()
    print("Surface points (faces):")
    for i, srf in enumerate(data["srf_pts"]):
        print(f"  [{i}] {srf['name']}: {len(srf['points'])} points")


if __name__ == "__main__":
    main()
