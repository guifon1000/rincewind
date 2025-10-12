# TODO: Unit Tests for Rincewind

## Overview
This document outlines the plan for implementing comprehensive unit tests for the Rincewind geometric computation library. The goal is to ensure code reliability, catch regressions, and facilitate future development.

## Current State
- **Existing Tests**: Several test scripts exist in `test/` directory (e.g., `test_triangle.py`, `test_house.py`, etc.)
- **Issues**: Current tests are more like integration/demo scripts than proper unit tests
  - Mix of multiple test scenarios in single files
  - No assertions or pass/fail validation
  - Require manual inspection of output/plots
  - Use `sys.exit()` which prevents running full test suites
  - Not automated (require matplotlib windows to be closed)

## Testing Framework Setup

### 1. Choose Testing Framework
- **Recommendation**: Use `pytest` (modern, flexible, good reporting)
- **Alternative**: `unittest` (stdlib, no dependencies)
- **Installation**: `pip install pytest pytest-cov`

### 2. Directory Structure
```
rincewind/
├── tests/                      # New directory for proper unit tests
│   ├── __init__.py
│   ├── conftest.py            # Shared fixtures
│   ├── test_geometry/
│   │   ├── __init__.py
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
│   │   ├── __init__.py
│   │   ├── test_basic_ops.py  # dot, cross, distance, angle
│   │   ├── test_intersections.py
│   │   ├── test_projections.py
│   │   └── test_transformations.py
│   ├── test_analytic_functions/
│   │   ├── __init__.py
│   │   └── test_bezier.py
│   └── test_io_functions/
│       ├── __init__.py
│       └── test_io.py
├── test/                       # Keep existing demo/integration scripts
│   └── ...                     # Rename to examples/ or demos/?
└── pytest.ini                  # Pytest configuration
```

### 3. Configuration Files

#### `pytest.ini`
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

#### `.coveragerc` (optional)
```ini
[run]
omit =
    */tests/*
    */test/*
    */__pycache__/*
    */site-packages/*
```

## Unit Test Implementation Plan

### Phase 1: Core Geometry Classes (Priority: HIGH)

#### 1.1 Point (`test_geometry/test_point.py`)
- [ ] **Creation & Initialization**
  - From list/tuple
  - From individual coordinates
  - Edge cases (empty, wrong dimensions)

- [ ] **Basic Operations**
  - Addition with vectors: `point + vector`
  - Subtraction: `point1 - point2` → vector
  - Negation: `-point`
  - Equality comparison with tolerance

- [ ] **Methods**
  - `to_array()`: conversion to numpy array
  - `express_in_frame_plane()`: projection onto frame
  - `add_to_ax()`: plotting (mock test)

- [ ] **Edge Cases**
  - Zero coordinates
  - Very large/small coordinates
  - Numerical precision issues

```python
# Example test structure
def test_point_addition_with_vector():
    p = Point([1.0, 2.0, 3.0])
    v = Vector([0.5, 0.5, 0.5])
    result = p + v
    assert isinstance(result, Point)
    assert result[0] == pytest.approx(1.5)
    assert result[1] == pytest.approx(2.5)
    assert result[2] == pytest.approx(3.5)
```

#### 1.2 Vector (`test_geometry/test_vector.py`)
- [ ] **Creation & Initialization**
  - From list/tuple
  - From two points

- [ ] **Vector Operations**
  - Magnitude: `vector.norm`
  - Unit vector: `vector.unit()`
  - Addition: `v1 + v2`
  - Subtraction: `v1 - v2`
  - Scalar multiplication: `scalar * vector`
  - Dot product (via geo_functions)
  - Cross product (via geo_functions)

- [ ] **Special Vectors**
  - Zero vector
  - Unit vectors along axes
  - Normalization of zero vector (should handle gracefully)

#### 1.3 Line (`test_geometry/test_line.py`)
- [ ] **Creation**
  - From point + vector
  - From two points

- [ ] **Methods**
  - `parameter_point()`: get point at parameter t
  - Equality testing: `line1 == line2`

- [ ] **Edge Cases**
  - Parallel lines
  - Coincident lines
  - Lines with zero direction (should error or handle)

#### 1.4 Segment (`test_geometry/test_segment.py`)
- [ ] **Creation**
  - From two points

- [ ] **Methods**
  - `line()`: convert to infinite line
  - Negation: `segment[::-1]`
  - Equality: overlapping segments
  - `add_to_ax()`: visualization

- [ ] **Edge Cases**
  - Zero-length segment
  - Very short segments
  - Parallel/overlapping segments

#### 1.5 Plane (`test_geometry/test_plane.py`)
- [ ] **Creation**
  - From 3 points
  - From point + normal vector
  - From 4 coefficients [a, b, c, d]

- [ ] **Properties**
  - `normal_vector`: unit normal

- [ ] **Methods**
  - `reflection_matrix()`: compute reflection matrix
  - `reflect_point()`: reflect point across plane
  - `reflect_vector()`: reflect vector
  - `reflect_frame()`: reflect coordinate frame

- [ ] **Edge Cases**
  - Collinear points (should error)
  - Zero normal vector
  - Point on plane vs. off plane

#### 1.6 Triangle (`test_geometry/test_triangle.py`)
- [ ] **Creation & Properties**
  - From 3 points
  - `cg`: center of gravity
  - `normal`: surface normal
  - `circumcenter`: circumscribed circle center
  - `orthocenter`: orthocenter
  - `area`: triangle area
  - `aspect_ratio`: shape quality metric
  - Reverse orientation: `-triangle`

- [ ] **Methods**
  - `frame()`: coordinate frame on triangle
  - `add_to_ax()`: visualization

- [ ] **Edge Cases**
  - Degenerate (collinear points)
  - Very flat triangles
  - Right triangles, equilateral triangles
  - Numerical precision near-degenerate cases

#### 1.7 Frame (`test_geometry/test_frame.py`)
- [ ] **Creation**
  - From origin + 3 basis vectors

- [ ] **Methods**
  - Coordinate transformations
  - Basis orthonormality checks

- [ ] **Edge Cases**
  - Non-orthogonal basis (should validate?)
  - Non-unit basis vectors

#### 1.8 Quaternion (`test_geometry/test_quaternion.py`)
- [ ] **Creation & Properties**
  - From 4 components (w, x, y, z)
  - `scalar`: scalar part
  - `vector`: vector part
  - `conjugate`: quaternion conjugate
  - `unit()`: normalize quaternion
  - `inverse()`: multiplicative inverse

- [ ] **Operations**
  - Multiplication: `q1 * q2`
  - Scalar multiplication: `scalar * q`
  - Division: `q1 / q2`
  - Power: `q ** n`
  - Magnitude: `abs(q)`

- [ ] **Conversions**
  - `to_matrix()`: convert to rotation matrix
  - `matrix_to_quaternion()`: convert from rotation matrix
  - Round-trip conversion: matrix → quat → matrix

- [ ] **Known Rotations**
  - 90° rotation around X, Y, Z axes
  - 180° rotations
  - Identity rotation
  - Quaternion interpolation (if implemented)

### Phase 2: Geometric Functions (Priority: HIGH)

#### 2.1 Basic Operations (`test_geo_functions/test_basic_ops.py`)
- [ ] **dot()**: dot product
  - Orthogonal vectors → 0
  - Parallel vectors → product of magnitudes
  - Commutative property

- [ ] **cross()**: cross product
  - Right-hand rule verification
  - Anti-commutative: `cross(a,b) = -cross(b,a)`
  - Orthogonality: result perpendicular to inputs
  - Zero for parallel vectors

- [ ] **distance()**: distance between points
  - Zero distance for same point
  - Commutative
  - Triangle inequality
  - Known distances (e.g., unit cube vertices)

- [ ] **angle()**: angle between vectors
  - Orthogonal → π/2
  - Parallel → 0 or π
  - With plane normal (signed angle)
  - Range validation [0, π] or [-π, π]

#### 2.2 Intersection Tests (`test_geo_functions/test_intersections.py`)
- [ ] **intersect_2_lines()**
  - Intersecting lines
  - Parallel lines → None
  - Skew lines (3D) → closest point or None
  - Coincident lines

- [ ] **intersect_2_segments()**
  - Crossing segments
  - Touching at endpoints
  - Parallel/non-intersecting
  - Overlapping segments
  - T-intersections

- [ ] **intersect_3_planes()**
  - Single point intersection
  - Parallel planes → None
  - Coincident planes
  - Line of intersection (two parallel, one crossing)

#### 2.3 Projection Tests (`test_geo_functions/test_projections.py`)
- [ ] **closest_point_on_line()**
  - Point projects onto line
  - Point on line → same point
  - Distance verification

- [ ] **closest_point_on_plane()**
  - Point projects onto plane
  - Point on plane → same point
  - Distance verification

- [ ] **is_on_line()**: point-line containment
  - Point on line
  - Point off line
  - Tolerance testing

- [ ] **is_on_plane()**: point-plane containment
  - Point on plane
  - Point off plane
  - Tolerance testing

- [ ] **is_on_segment()**: point-segment containment
  - Point on segment interior
  - Point at endpoints
  - Point on extended line but not segment

#### 2.4 Complex Operations (`test_geo_functions/test_transformations.py`)
- [ ] **cut_triangle_by_plane()**
  - Triangle entirely above/below → no cut
  - Triangle intersected → 2 or 3 polygons
  - Triangle on plane
  - Vertex on plane
  - Edge on plane

- [ ] **rotation_minimized_frame()** (if exists)
  - Frame propagation along curve
  - Twist minimization

- [ ] **classify_polylines()** (if exists)
  - Interior vs. exterior contours
  - Hole detection

### Phase 3: Polylines & Triangulation (Priority: MEDIUM)

#### 3.1 Polyline2D (`test_geometry/test_polyline2d.py`)
- [ ] **Creation & Properties**
  - From list of points
  - `is_closed`: check closure
  - `orientation`: clockwise vs. counterclockwise

- [ ] **Methods**
  - `close()`: close polyline
  - `to_frame()`: transform to 3D frame
  - `orient()`: enforce orientation
  - `segments`: get list of segments
  - `auto_intersects()`: self-intersection check

- [ ] **Special Classes**
  - `Circle`: circular polyline
  - `NamedPolyline`: named boundary sections
  - `NamedCartesianBox`: rectangular boundary

#### 3.2 Polyline3D (`test_geometry/test_polyline3d.py`)
- [ ] **Creation & Properties**
  - From list of 3D points
  - `is_closed()`: check closure
  - `is_planar()`: coplanarity test
  - `gravity_center`: centroid

- [ ] **Methods**
  - `remove_duplicates()`: eliminate duplicate points
  - `triangulate_planar()`: triangulate planar polyline
  - `express_in_frame_plane()`: project to 2D
  - `list_of_segments()`: decompose into segments
  - `integrate_lengthes()`: compute perimeter

#### 3.3 Triangulation (`test_geometry/test_triangulation.py`)
- [ ] **Creation**
  - From vertices + faces
  - Physical groups/tags

- [ ] **Properties**
  - `cg`: center of gravity
  - `bounding_box`: axis-aligned bounds
  - `triangle_list`: get all triangles

- [ ] **Methods**
  - `translate()`: translate mesh
  - `reverse()`: flip normals
  - `reorient_convex()`: orient convex mesh
  - `refine_2()`: subdivide triangles
  - `vertex_normals()`: compute vertex normals

- [ ] **I/O**
  - `write_obj_file()`: OBJ export
  - `write_stl_file()`: STL export
  - `read_msh_file()`: Gmsh import
  - `read_stl_file()`: STL import
  - Round-trip tests: write → read → compare

### Phase 4: Advanced Geometry (Priority: MEDIUM)

#### 4.1 Extrusion (`test_geometry/test_extrusion.py`)
- [ ] **Creation**
  - From 2D profile + path + frames
  - Swept surfaces

- [ ] **Methods**
  - `triangulate()`: generate surface mesh
  - `classify_caps()`: identify end caps
  - Geometry validation

#### 4.2 ParametricCurve3D (`test_geometry/test_parametric_curve3d.py`)
- [ ] **Creation**
  - From parametric functions
  - Spline interpolation

- [ ] **Methods**
  - `parameter_point()`: evaluate at parameter
  - `tangent_vector()`: compute tangent
  - `discretize()`: sample curve
  - `frame_array()`: generate frame sequence

### Phase 5: Analytic & I/O Functions (Priority: LOW)

#### 5.1 Bezier Functions (`test_analytic_functions/test_bezier.py`)
- [ ] **de_casteljau()**: evaluation algorithm
  - Known control points → known curve points
  - Endpoints match control points

- [ ] **bezier_function()**: function generator
  - Smooth interpolation
  - Derivative continuity

- [ ] **piecewise_bezier_polyline()**: piecewise curves
  - Continuity between pieces
  - Fillet generation

#### 5.2 I/O Functions (`test_io_functions/test_io.py`)
- [ ] **pretty_print()**: formatted output
  - Mock stdout testing

- [ ] File I/O (if not covered in Triangulation)
  - JSON, OBJ, STL format validation

## Testing Best Practices

### General Principles
1. **Test Independence**: Each test should run independently
2. **Reproducibility**: Use fixed seeds for random tests
3. **Fast Execution**: Unit tests should run quickly (< 1s each)
4. **Clear Names**: `test_<functionality>_<scenario>_<expected_result>`
5. **Single Assertion Focus**: Test one thing at a time (when possible)
6. **Use Fixtures**: Share common setup via pytest fixtures

### Numerical Testing
```python
import pytest
import numpy as np

# For floating point comparisons
assert value == pytest.approx(expected, rel=1e-6, abs=1e-9)

# For numpy arrays
np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-9)
```

### Fixture Examples
```python
# conftest.py
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

@pytest.fixture
def basis_vectors():
    """Standard basis vectors."""
    return {
        'x': Vector([1, 0, 0]),
        'y': Vector([0, 1, 0]),
        'z': Vector([0, 0, 1])
    }
```

### Parameterized Tests
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

### Property-Based Testing (Advanced)
```python
from hypothesis import given, strategies as st

@given(
    x=st.floats(min_value=-1000, max_value=1000),
    y=st.floats(min_value=-1000, max_value=1000),
    z=st.floats(min_value=-1000, max_value=1000)
)
def test_point_creation_always_succeeds(x, y, z):
    """Point creation should work for any reasonable floats."""
    p = Point([x, y, z])
    assert len(p) == 3
```

### Mocking Visualization
```python
from unittest.mock import Mock, patch

def test_triangle_add_to_ax():
    """Test that add_to_ax calls matplotlib correctly."""
    triangle = Triangle([Point([0,0,0]), Point([1,0,0]), Point([0,1,0])])
    mock_ax = Mock()

    triangle.add_to_ax(mock_ax)

    # Verify plot_trisurf was called
    mock_ax.plot_trisurf.assert_called_once()
```

## Integration & Regression Tests

### Integration Tests (in `tests/integration/`)
- [ ] **Full workflow tests**
  - Create geometry → transform → triangulate → export
  - Import mesh → modify → re-export → compare

- [ ] **Example-based tests**
  - Convert existing demo scripts to assertions
  - `test_house_geometry.py`: house example validation
  - `test_zome_structures.py`: geodesic dome validation

### Regression Tests
- [ ] **Known bugs**: Create tests that verify fixes
- [ ] **Edge cases**: Document and test problematic scenarios
- [ ] **Performance**: Track performance of critical operations

## Continuous Integration Setup

### GitHub Actions / GitLab CI
```yaml
# .github/workflows/tests.yml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: [3.8, 3.9, 3.10, 3.11]

    steps:
    - uses: actions/checkout@v3
    - name: Set up Python ${{ matrix.python-version }}
      uses: actions/setup-python@v4
      with:
        python-version: ${{ matrix.python-version }}
    - name: Install dependencies
      run: |
        pip install -e .
        pip install pytest pytest-cov
    - name: Run tests
      run: pytest --cov=rincewind --cov-report=xml
    - name: Upload coverage
      uses: codecov/codecov-action@v3
```

## Documentation Integration

### Docstring Tests
- [ ] Add docstring examples to major classes/functions
- [ ] Use `pytest --doctest-modules` to validate examples

### Test Documentation
- [ ] Document complex test setups
- [ ] Explain tolerance choices for numerical tests
- [ ] Reference mathematical properties being tested

## Migration Strategy

### Step 1: Set Up Infrastructure (Week 1)
1. Install pytest and configure
2. Create directory structure
3. Set up fixtures and utilities
4. Write first 5-10 basic tests as examples

### Step 2: Core Geometry (Week 2-3)
1. Point, Vector (fundamental building blocks)
2. Line, Segment, Plane
3. Triangle, Frame
4. Basic geo_functions

### Step 3: Polylines & Triangulation (Week 4)
1. Polyline2D, Polyline3D
2. Triangulation and I/O

### Step 4: Advanced & Integration (Week 5+)
1. Quaternion, Extrusion, Parametric curves
2. Integration tests
3. Performance tests
4. Documentation

### Incremental Approach
- Start with **most critical** or **most frequently used** components
- Each PR should add tests along with code changes
- Gradually increase coverage target: 50% → 70% → 85%+

## Success Metrics

- [ ] **Coverage**: Aim for >85% line coverage, >70% branch coverage
- [ ] **Speed**: Full test suite runs in <30 seconds
- [ ] **Reliability**: All tests pass consistently across platforms
- [ ] **Maintainability**: New contributors can easily add tests
- [ ] **Documentation**: All public APIs have docstring examples

## Tools & Commands

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

# Run and stop at first failure
pytest -x

# Run last failed tests
pytest --lf

# Parallel execution (with pytest-xdist)
pytest -n auto
```

## Notes & Considerations

### Current Codebase Issues to Address
1. **Python 2 vs 3**: Some code still has Python 2 remnants (now mostly fixed)
2. **Import structure**: Absolute vs. relative imports (now fixed)
3. **Error handling**: Many functions don't validate input or handle edge cases
4. **Type hints**: Consider adding type annotations for better testing
5. **Matplotlib dependencies**: Mock for unit tests, keep for integration tests

### Future Enhancements
- **Type checking**: Add mypy for static type checking
- **Benchmarking**: Add performance regression tests with pytest-benchmark
- **Mutation testing**: Use mutmut to verify test quality
- **Documentation coverage**: Track docstring completeness

---

**Last Updated**: 2025-10-12
**Status**: Planning Phase
**Next Steps**: Set up pytest infrastructure and write first 10 tests for Point/Vector classes
