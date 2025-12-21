# Rincewind

A Python library for 3D geometric modeling and computational geometry with a focus on parametric design and engineering applications.

## Overview

Rincewind provides a comprehensive toolkit for creating, manipulating, and analyzing 3D geometric objects. It's particularly well-suited for:

- Ship hull design and modeling
- Aerodynamic surface generation (wings, airfoils)
- Parametric curve and surface creation
- Triangulation and mesh generation
- Extrusion-based modeling
- Carpentry 
## Features

- **Core Geometric Primitives**: Points, vectors, lines, planes, frames
- **Parametric Curves and Surfaces**: Bezier curves, splines, UV surfaces
- **Advanced Modeling Tools**: Extrusion, triangulation, mesh operations
- **Domain-Specific Generators**: Ship hulls, wings, architectural elements
- **Import/Export Capabilities**: Support for STL, OBJ, JSON, MSH formats
- **Visualization Utilities**: Matplotlib integration for displaying models

## Structure

- `geometry/`: Core geometric primitives and operations
- `analytic_functions/`: Mathematical tools for curve generation
- `modelers/`: Domain-specific object generators
- `geo_functions/`: Utility functions for geometric operations
- `io_functions/`: Import/export functionality
- `visualization/`: Visualization tools

## Examples

Check the `test/` directory for usage examples:

- Wing modeling (test_wing.py)
- Ship hull generation (wigley_hull.py)
- Extrusion-based modeling (test_extrusion.py)
- Architectural modeling (test_house.py)

## Dependencies

- NumPy
- Matplotlib
- SciPy
- pygmsh (optional, for advanced mesh operations)

## Installation

### Quick Start

```bash
git clone https://github.com/guifon1000/rincewind.git
cd rincewind
pip install -e .
```

### Installation Modes

**Development mode** (recommended for development):
```bash
# Install with dev dependencies (pytest, pytest-cov)
pip install -e ".[dev]"
```

**Production mode** (minimal dependencies):
```bash
# Install only core dependencies (numpy, matplotlib)
pip install .
```

### Verify Installation

```bash
# Check installation
python -c "import rincewind; print(rincewind.__file__)"

# Run tests
pytest test/ -v
```

### Configuration

The project uses modern Python standards:
- **pyproject.toml**: Main configuration file (metadata, dependencies, build system)
- **setup.py**: Minimal legacy compatibility (calls setuptools.setup())
- **Requires Python 3.8+**

## Usage

```python
from rincewind.geometry import Point, Vector, Triangulation
from rincewind.modelers.ship_hulls import wigley_hull

# Create basic geometry
point = Point(1, 2, 3)
vector = Vector(1, 0, 0)

# Generate a ship hull
hull = wigley_hull.create_wigley_hull(length=100, breadth=20, draft=10)

# Export to STL
hull.to_stl("wigley_hull.stl")
```

## License

You can do what you want with this tool.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.