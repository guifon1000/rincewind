# RinceWind - Complete Documentation

*A Python library for 3D geometric modeling and computational geometry with a focus on parametric design and engineering applications.*

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Export System Documentation](#export-system-documentation)
3. [Extending the Export System](#extending-the-export-system)
4. [Testing Strategy](#testing-strategy)

---

## Project Overview

### About RinceWind

RinceWind provides a comprehensive toolkit for creating, manipulating, and analyzing 3D geometric objects. It's particularly well-suited for:

- Ship hull design and modeling
- Aerodynamic surface generation (wings, airfoils)
- Parametric curve and surface creation
- Triangulation and mesh generation
- Extrusion-based modeling
- Carpentry

### Core Features

- **Geometric Primitives**: Points, vectors, lines, planes, frames
- **Parametric Curves and Surfaces**: Bezier curves, splines, UV surfaces
- **Advanced Modeling Tools**: Extrusion, triangulation, mesh operations
- **Domain-Specific Generators**: Ship hulls, wings, architectural elements
- **Import/Export Capabilities**: Support for STL, OBJ, JSON, MSH formats
- **Visualization Utilities**: Matplotlib integration for displaying models

### Project Structure

The project follows modern Python standards with clear separation of concerns:

```
rincewind/                          # Project root
├── pyproject.toml                  # Configuration (Python 3.8+)
├── setup.py                        # Minimal setup (legacy compatibility)
├── README.md                       # Quick start guide
├── DOCUMENTATION.md                # Full documentation
│
├── rincewind/                      # Package directory
│   ├── __init__.py
│   ├── geometry/                   # Core geometric primitives
│   ├── analytic_functions/         # Mathematical tools
│   ├── modelers/                   # Domain-specific generators
│   ├── geo_functions/              # Utility functions
│   ├── io_functions/               # Import/export
│   ├── visualization/              # Visualization tools
│   ├── document/                   # GDL export system
│   │   ├── exporter.py
│   │   └── gdl_parser.py
│   └── *.json                      # Assets
│
├── test/                           # Test suite
│   ├── test_exporter.py           # Core export tests (19)
│   └── test_export_reference_pyramid.py  # Reference tests (9)
│
└── examples/                       # Usage examples
    └── pyramid_export_demo.py
```

### Installation

**Development mode** (recommended for contributing):
```bash
git clone https://github.com/guifon1000/rincewind.git
cd rincewind
pip install -e ".[dev]"
```

**Production mode**:
```bash
git clone https://github.com/guifon1000/rincewind.git
cd rincewind
pip install .
```

The project uses `pyproject.toml` for configuration following modern Python standards (PEP 517/518). The minimal `setup.py` is provided for backward compatibility only.

**Verify installation:**
```bash
pytest test/ -v
```

### Dependencies

**Core** (always installed):
- NumPy
- Matplotlib

**Development** (optional, install with `pip install -e ".[dev]"`):
- pytest (≥7.0)
- pytest-cov (≥4.0)

**Optional** (not included):
- SciPy (used by some geometry modules)
- pygmsh (for advanced mesh operations)

### Usage Example

```python
from rincewind.geometry import Point, Vector
from rincewind.modelers.ship_hulls import wigley_hull

# Create basic geometry
point = Point(1, 2, 3)
vector = Vector(1, 0, 0)

# Generate a ship hull
hull = wigley_hull.create_wigley_hull(length=100, breadth=20, draft=10)

# Export to STL
hull.to_stl("wigley_hull.stl")
```

---

## Export System Documentation

### Overview

The export system converts RinceWind geometry objects into GDL (Geometric Description Language) JSON format, suitable for:
- Storing geometry data
- Transmitting to other systems (e.g., Rhino3D parser)
- LLM-friendly representation
- Documentation and visualization

### Supported Geometry Types

#### 1. GDLPoint - 3D Point

```python
@dataclass
class GDLPoint:
    x: float
    y: float
    z: float
    name: Optional[str] = None
```

**JSON Example:**
```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "name": "origin"
}
```

**Usage:**
```python
exporter = RincewindExporter()
point = Point([1.0, 2.0, 3.0])
idx = exporter.add_point(point, "my_point")  # Returns: 0
```

#### 2. GDLLine - Line Segment

```python
@dataclass
class GDLLine:
    start: int  # Index in points array
    end: int    # Index in points array
    name: Optional[str] = None
```

**JSON Example:**
```json
{
  "start": 0,
  "end": 1,
  "name": "edge"
}
```

**Usage:**
```python
from rincewind.geometry.Segment import Segment

seg = Segment([Point([0, 0, 0]), Point([1, 1, 1])])
idx = exporter.add_segment(seg, "edge1")  # Returns: 0
```

#### 3. GDLPolyline - Multi-Point Path

```python
@dataclass
class GDLPolyline:
    points: List[int]  # Indices in points array (2+ points)
    name: Optional[str] = None
```

**JSON Example:**
```json
{
  "points": [0, 1, 2, 3, 0],
  "name": "square"
}
```

**Usage:**
```python
from rincewind.geometry.Polyline3D import Polyline3D

pts = [Point([0, 0, 0]), Point([1, 0, 0]), Point([1, 1, 0])]
polyline = Polyline3D(pts)
idx = exporter.add_polyline(polyline, "triangle")  # Returns: 0
```

#### 4. GDL_SrfPt - Surface/Face Points

```python
@dataclass
class GDL_SrfPt:
    points: List[int]  # 3-4 indices (triangle or quad)
    name: Optional[str] = None
```

**JSON Example:**
```json
{
  "points": [0, 1, 2],
  "name": "triangular_face"
}
```

**Usage:**
```python
pts = [Point([0, 0, 0]), Point([1, 0, 0]), Point([0.5, 1, 0])]
idx = exporter.add_srf_pt(pts, "triangle_face")  # Returns: 0
```

### Complete Export Workflow

```python
from rincewind.document.exporter import RincewindExporter
from rincewind.document.gdl_parser import JSONGDLParser

# Create exporter
exporter = RincewindExporter()

# Add geometry
p1 = Point([0.0, 0.0, 0.0])
p2 = Point([1.0, 0.0, 0.0])
p3 = Point([1.0, 1.0, 0.0])

exporter.add_point(p1, "p1")
exporter.add_point(p2, "p2")
exporter.add_point(p3, "p3")

from rincewind.geometry.Segment import Segment
exporter.add_segment(Segment([p1, p2]), "edge1")
exporter.add_segment(Segment([p2, p3]), "edge2")

from rincewind.geometry.Polyline3D import Polyline3D
exporter.add_polyline(Polyline3D([p1, p2, p3]), "triangle")
exporter.add_srf_pt([p1, p2, p3], "face")

# Export to JSON
json_str = exporter.to_json_string()
exporter.save_json("output.json")

# Parse back
parser = JSONGDLParser()
context = parser.parse(json_str)
context.summary()
```

### JSON Output Format

```json
{
  "points": [
    {"x": 0.0, "y": 0.0, "z": 0.0, "name": "p1"},
    {"x": 1.0, "y": 0.0, "z": 0.0, "name": "p2"},
    {"x": 1.0, "y": 1.0, "z": 0.0, "name": "p3"}
  ],
  "lines": [
    {"start": 0, "end": 1, "name": "edge1"},
    {"start": 1, "end": 2, "name": "edge2"}
  ],
  "polylines": [
    {"points": [0, 1, 2], "name": "triangle"}
  ],
  "srf_pts": [
    {"points": [0, 1, 2], "name": "face"}
  ]
}
```

### API Reference

#### RincewindExporter Class

**Methods:**

- `add_point(point: Point, name: Optional[str] = None) -> int`
  - Adds a Point to the context
  - Returns the index of the point

- `add_segment(segment: Segment, name: Optional[str] = None) -> int`
  - Adds a Segment (line) to the context
  - Automatically creates endpoint points
  - Returns the index of the line

- `add_polyline(polyline: Polyline3D, name: Optional[str] = None) -> int`
  - Adds a Polyline3D to the context
  - Automatically creates all points
  - Returns the index of the polyline

- `add_srf_pt(pts: List[Point], name: Optional[str] = None) -> int`
  - Adds surface points (3-4 points for faces)
  - Returns the index in the srf_pts array

- `to_dict() -> Dict[str, Any]`
  - Returns all geometry as a Python dictionary

- `to_json_string(indent: int = 2) -> str`
  - Returns all geometry as a formatted JSON string

- `save_json(filepath: Union[str, Path]) -> None`
  - Saves geometry to a JSON file

- `reset() -> None`
  - Clears all data and starts fresh

- `summary() -> None`
  - Prints a summary of all geometry

#### JSONGDLParser Class

**Methods:**

- `parse(json_data: Union[str, dict, list]) -> GeometryContext`
  - Parses JSON GDL data
  - Accepts JSON strings, dictionaries, or lists

- `parse_file(filepath: str) -> GeometryContext`
  - Parses JSON GDL from a file

### Design Principles

1. **Simplicity**: Only 4 geometry types, focused on actual usage
2. **Index References**: All geometry uses integer indices (efficient, JSON-friendly)
3. **Optional Naming**: Names are metadata for human readability
4. **JSON Format**: All exports are valid JSON for external tool compatibility
5. **Round-Trip Consistency**: Export → Parse → Export produces identical results

---

## Extending the Export System

### Quick Start Checklist

Adding a new geometry type requires changes to:

- [ ] **gdl_parser.py** - Define dataclass + update GeometryContext
- [ ] **exporter.py** - Optional: add_* method
- [ ] **test/test_exporter.py** - Add tests

### Step-by-Step Guide: Adding GDLCircle

#### Step 1: Define Dataclass (gdl_parser.py)

```python
@dataclass
class GDLCircle:
    center: int              # Index in points array
    radius: float
    normal: List[float]      # [x, y, z]
    name: Optional[str] = None

    def __repr__(self):
        if self.name:
            return f"Circle({self.name}: r={self.radius})"
        return f"Circle(r={self.radius})"
```

#### Step 2: Add to GeometryContext (gdl_parser.py)

**In `__init__`:**
```python
self.circles = []
```

**Add accessor:**
```python
def add_circle(self, obj: GDLCircle):
    self.circles.append(obj)
```

**Update `summary()`:**
```python
print(f"Circles:   {len(self.circles)}")
```

**Update `to_json()`:**
```python
"circles": [self._dict_without_none_name(c) for c in self.circles],
```

#### Step 3: Update Parser (gdl_parser.py)

**In `parse()` key detection:**
```python
elif any(key in data for key in [..., 'circles']):
```

**In `_parse_geometry_dict()`:**
```python
for circle_data in data.get('circles', []):
    circle = GDLCircle(
        center=circle_data['center'],
        radius=circle_data['radius'],
        normal=circle_data['normal'],
        name=circle_data.get('name')
    )
    self.context.add_circle(circle)
```

**In `_parse_object()`:**
```python
elif obj_type == 'circle':
    circle = GDLCircle(
        center=obj['center'],
        radius=obj['radius'],
        normal=obj['normal'],
        name=obj.get('name')
    )
    self.context.add_circle(circle)
```

#### Step 4: Add to Exporter (exporter.py - Optional)

```python
def add_circle(self, center: Point, radius: float, normal: List[float],
               name: Optional[str] = None) -> int:
    center_idx = self.add_point(center)
    gdl_circle = GDLCircle(
        center=center_idx,
        radius=float(radius),
        normal=list(normal),
        name=name
    )
    idx = len(self.context.circles)
    self.context.add_circle(gdl_circle)
    return idx
```

#### Step 5: Add Tests (test/test_exporter.py)

```python
class TestCircleExport:
    def test_export_circle(self):
        """Test exporting a circle."""
        exporter = RincewindExporter()
        exporter.add_circle(
            center=Point([0, 0, 0]),
            radius=5.0,
            normal=[0, 0, 1],
            name="base_circle"
        )
        data = exporter.to_dict()
        assert len(data["circles"]) == 1
        assert data["circles"][0]["radius"] == 5.0

    def test_circle_roundtrip(self):
        """Test export → parse → export consistency."""
        exporter = RincewindExporter()
        exporter.add_circle(
            center=Point([1, 2, 3]),
            radius=10.0,
            normal=[1, 0, 0],
            name="test"
        )

        json_1 = exporter.to_dict()
        parser = JSONGDLParser()
        ctx = parser.parse(json_1)
        json_2 = ctx.to_json()

        assert len(json_1["circles"]) == len(json_2["circles"])
        assert json_1["circles"][0]["radius"] == json_2["circles"][0]["radius"]
```

#### Step 6: Run Tests

```bash
pytest test/test_exporter.py -v
pytest test/test_export_reference_pyramid.py -v
```

### Design Patterns

#### Pattern 1: Point-Referenced Geometry (Arc, Circle, BSpline)
```python
@dataclass
class GDLArc:
    center: int              # Reference to point
    radius: float
    start_angle: float
    end_angle: float
    normal: List[float]      # [x, y, z]
    name: Optional[str] = None
```

#### Pattern 2: Geometry-Referenced Geometry (Face, Surface)
```python
@dataclass
class GDLFace:
    boundary: int            # Reference to loop
    inner: Optional[List[int]] = None  # References to inner loops
    plane: Optional[int] = None  # Reference to plane
    name: Optional[str] = None
```

#### Pattern 3: Parameter-Based Geometry (Solid, Surface)
```python
@dataclass
class GDLSurface:
    surface_type: str        # "loft", "sweep", "extrude"
    params: Dict[str, Any]   # Flexible parameters
    name: Optional[str] = None
```

### Validation Checklist

When adding a new geometry type, ensure:

- [ ] Dataclass defined with all required fields
- [ ] `name: Optional[str] = None` field included
- [ ] `__repr__()` method implemented
- [ ] Storage array added to `GeometryContext.__init__()`
- [ ] `add_*()` accessor method added
- [ ] `summary()` updated
- [ ] `to_json()` updated
- [ ] Parser key detection updated
- [ ] `_parse_geometry_dict()` updated
- [ ] `_parse_object()` updated
- [ ] Optional: `add_*()` method in `RincewindExporter`
- [ ] Tests for basic export
- [ ] Tests for round-trip consistency (export → parse → export)
- [ ] All existing tests still pass

---

## Testing Strategy

### Overview

This document outlines the comprehensive testing strategy for RinceWind. The goal is to ensure code reliability, catch regressions, and facilitate future development.

### Current State

**Existing Tests:**
- Several test scripts exist in `test/` directory (e.g., `test_triangle.py`, `test_house.py`)
- More like integration/demo scripts than proper unit tests
- Mix of multiple scenarios in single files
- No assertions or pass/fail validation
- Require manual inspection of output/plots

**Issues to Address:**
1. Tests lack assertions for automated validation
2. Cannot run full test suites (sys.exit() blocks)
3. Require matplotlib windows to be closed manually
4. Not suitable for continuous integration

### Testing Framework Setup

#### Framework: pytest

Install pytest and pytest-cov:
```bash
pip install pytest pytest-cov
```

#### Directory Structure

```
rincewind/
├── test/                       # Keep existing demo/integration scripts
├── tests/                      # New directory for proper unit tests
│   ├── __init__.py
│   ├── conftest.py            # Shared fixtures
│   ├── test_geometry/
│   │   ├── test_point.py
│   │   ├── test_vector.py
│   │   ├── test_line.py
│   │   ├── test_segment.py
│   │   ├── test_plane.py
│   │   ├── test_triangle.py
│   │   ├── test_frame.py
│   │   ├── test_quaternion.py
│   │   ├── test_polyline2d.py
│   │   ├── test_polyline3d.py
│   │   ├── test_triangulation.py
│   │   └── test_extrusion.py
│   ├── test_geo_functions/
│   │   ├── test_basic_ops.py
│   │   ├── test_intersections.py
│   │   ├── test_projections.py
│   │   └── test_transformations.py
│   ├── test_analytic_functions/
│   │   └── test_bezier.py
│   ├── test_io_functions/
│   │   └── test_io.py
│   └── integration/
│       └── test_workflows.py
└── pytest.ini                  # Pytest configuration
```

#### Configuration: pytest.ini

```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts =
    -v
    --tb=short
    --cov=rincewind
    --cov-report=html
    --cov-report=term-missing
```

### Test Implementation Plan

#### Phase 1: Core Geometry Classes (HIGH PRIORITY)

**Point, Vector, Line, Segment, Plane, Triangle, Frame, Quaternion**

Example test structure:
```python
def test_point_addition_with_vector():
    p = Point([1.0, 2.0, 3.0])
    v = Vector([0.5, 0.5, 0.5])
    result = p + v
    assert isinstance(result, Point)
    assert result[0] == pytest.approx(1.5)
    assert result[1] == pytest.approx(2.5)
    assert result[2] == pytest.approx(3.5)
```

#### Phase 2: Geometric Functions (HIGH PRIORITY)

**Dot product, Cross product, Distance, Angle, Intersections, Projections**

```python
@pytest.mark.parametrize("coords,expected_norm", [
    ([3, 4, 0], 5.0),
    ([1, 0, 0], 1.0),
    ([1, 1, 1], np.sqrt(3)),
])
def test_vector_norm(coords, expected_norm):
    v = Vector(coords)
    assert v.norm == pytest.approx(expected_norm)
```

#### Phase 3: Polylines & Triangulation (MEDIUM PRIORITY)

**Polyline2D, Polyline3D, Triangulation, I/O operations**

```python
def test_triangulation_stl_roundtrip():
    """Write STL → read back → compare."""
    tri = Triangulation([...])
    tri.write_stl_file("test.stl")
    tri2 = Triangulation.read_stl_file("test.stl")
    assert len(tri.vertices) == len(tri2.vertices)
```

#### Phase 4: Advanced Geometry (MEDIUM PRIORITY)

**Extrusion, ParametricCurve3D**

#### Phase 5: Analytic & I/O Functions (LOW PRIORITY)

**Bezier functions, Format converters**

### Testing Best Practices

#### Numerical Testing

```python
import pytest
import numpy as np

# For floating point comparisons
assert value == pytest.approx(expected, rel=1e-6, abs=1e-9)

# For numpy arrays
np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-9)
```

#### Fixtures (conftest.py)

```python
import pytest
from rincewind.geometry import Point, Vector, Triangle

@pytest.fixture
def unit_cube_vertices():
    """8 vertices of a unit cube."""
    return [
        Point([0, 0, 0]), Point([1, 0, 0]),
        Point([1, 1, 0]), Point([0, 1, 0]),
        Point([0, 0, 1]), Point([1, 0, 1]),
        Point([1, 1, 1]), Point([0, 1, 1]),
    ]

@pytest.fixture
def equilateral_triangle():
    """Equilateral triangle in XY plane."""
    h = np.sqrt(3) / 2
    return Triangle([
        Point([0, 0, 0]),
        Point([1, 0, 0]),
        Point([0.5, h, 0])
    ])
```

#### Mocking Visualization

```python
from unittest.mock import Mock

def test_triangle_add_to_ax():
    """Test that add_to_ax calls matplotlib correctly."""
    triangle = Triangle([
        Point([0, 0, 0]), Point([1, 0, 0]), Point([0, 1, 0])
    ])
    mock_ax = Mock()

    triangle.add_to_ax(mock_ax)

    # Verify plot_trisurf was called
    mock_ax.plot_trisurf.assert_called_once()
```

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=rincewind --cov-report=html

# Run specific test file
pytest tests/test_geometry/test_point.py

# Run specific test
pytest tests/test_geometry/test_point.py::test_point_addition

# Run tests matching pattern
pytest -k "triangle"

# Run with verbose output
pytest -v

# Stop at first failure
pytest -x

# Run last failed tests
pytest --lf

# Parallel execution (requires pytest-xdist)
pytest -n auto
```

### Integration Testing

**Full workflow tests:**
- Create geometry → transform → triangulate → export
- Import mesh → modify → re-export → compare

**Example-based tests:**
- Convert existing demo scripts to assertions
- Validate house geometry example
- Validate geodesic dome example

### Success Metrics

- [ ] **Coverage**: >85% line coverage, >70% branch coverage
- [ ] **Speed**: Full test suite runs in <30 seconds
- [ ] **Reliability**: All tests pass consistently across platforms
- [ ] **Maintainability**: New contributors can easily add tests
- [ ] **Documentation**: All public APIs have docstring examples

### Migration Strategy

**Week 1: Infrastructure**
- Install pytest and configure
- Create directory structure
- Set up fixtures and utilities

**Week 2-3: Core Geometry**
- Point, Vector (fundamental building blocks)
- Line, Segment, Plane
- Triangle, Frame

**Week 4: Polylines & Triangulation**
- Polyline2D, Polyline3D
- Triangulation and I/O

**Week 5+: Advanced & Integration**
- Quaternion, Extrusion, Parametric curves
- Integration tests
- Performance tests

---

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Guidelines

1. **Add tests** with new features
2. **Maintain coverage** above 85%
3. **Follow coding style** of existing codebase
4. **Document public APIs** with docstrings
5. **Validate round-trip consistency** for export operations

## License

You can do what you want with this tool.

---

**Last Updated**: 2025-12-21
**Maintained By**: RinceWind Contributors
