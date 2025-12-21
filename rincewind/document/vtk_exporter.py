
import vtk
from typing import List
from rincewind.geo_functions import distance




class VTKExporter:
    """
    Handles VTK file export for 3D visualization.
    
    VTK files can be opened in:
    - ParaView (free, powerful visualization)
    - Blender (with VTK plugin)
    - MayaVi
    - Custom VTK applications
    """
    
    @staticmethod
    def export_bars_to_vtk(bars: List['Bar'], filename: str = "carpentry_bars.vtk"):
        """
        Export bars as VTK unstructured grid.
        Each bar is represented as a hexahedral cell (rectangular prism).
        """
        # Create VTK data structures
        points = vtk.vtkPoints()
        cells = vtk.vtkCellArray()
        
        # Keep track of point indices
        point_index = 0
        
        # Process each bar
        for bar_idx, bar in enumerate(bars):
            # Get bar vertices (8 points for rectangular prism)
            vertices = bar.get_vertices()
            
            # Add points to VTK
            bar_point_indices = []
            for vertex in vertices:
                points.InsertNextPoint(vertex[0], vertex[1], vertex[2])
                bar_point_indices.append(point_index)
                point_index += 1
            
            # Create hexahedral cell (rectangular prism) 
            # VTK hexahedron vertex ordering is specific
            hex_cell = vtk.vtkHexahedron()
            
            # Map our vertex ordering to VTK hexahedron ordering
            # Our vertices: 4 corners at start, 4 corners at end
            # VTK hexahedron: specific ordering for proper rendering
            vtk_order = [0, 1, 2, 3, 4, 5, 6, 7]  # Direct mapping for now
            
            for i, vtk_idx in enumerate(vtk_order):
                hex_cell.GetPointIds().SetId(i, bar_point_indices[vtk_idx])
            
            cells.InsertNextCell(hex_cell)
        
        # Create unstructured grid
        ugrid = vtk.vtkUnstructuredGrid()
        ugrid.SetPoints(points)
        ugrid.SetCells(vtk.VTK_HEXAHEDRON, cells)
        
        # Add bar properties as cell data
        bar_ids = vtk.vtkIntArray()
        bar_ids.SetName("BarID")
        
        lengths = vtk.vtkFloatArray()
        lengths.SetName("Length")
        
        widths = vtk.vtkFloatArray()
        widths.SetName("Width")
        
        heights = vtk.vtkFloatArray()
        heights.SetName("Height")
        
        for i, bar in enumerate(bars):
            bar_ids.InsertNextValue(i)
            lengths.InsertNextValue(distance(bar.pt1, bar.pt2))
            widths.InsertNextValue(bar.section[0])
            heights.InsertNextValue(bar.section[1])
        
        ugrid.GetCellData().AddArray(bar_ids)
        ugrid.GetCellData().AddArray(lengths)
        ugrid.GetCellData().AddArray(widths)
        ugrid.GetCellData().AddArray(heights)
        
        # Write to file
        writer = vtk.vtkUnstructuredGridWriter()
        writer.SetFileName(filename)
        writer.SetInputData(ugrid)
        writer.Write()
        
        print(f"Exported {len(bars)} bars to {filename}")
    
    @staticmethod
    def export_cutting_planes_to_vtk(bars: List['Bar'], filename: str = "cutting_planes.vtk"):
        """
        Export cutting planes as VTK polygonal data.
        Useful for visualizing where cuts will be made.
        """
        points = vtk.vtkPoints()
        polys = vtk.vtkCellArray()
        
        plane_count = 0
        
        for bar in bars:
            for plane in bar.cuts:
                # Create a square plane for visualization
                plane_size = max(bar.section) * 3  # Make plane larger than bar

                # Get plane normal vector from rincewind Plane (it's in the first 3 elements)
                # Rincewind Plane format: [a, b, c, d] where ax + by + cz + d = 0
                normal = plane.normal_vector  # This returns a normalized Vector
                normal = np.array([normal[0], normal[1], normal[2]])

                # Calculate a point on the plane (origin)
                # For plane ax + by + cz + d = 0, we can find a point
                # We'll use the point closest to the world origin
                if abs(plane[2]) > 0.001:  # Use z component if non-zero
                    origin = np.array([0, 0, -plane[3] / plane[2]])
                elif abs(plane[1]) > 0.001:  # Use y component if non-zero
                    origin = np.array([0, -plane[3] / plane[1], 0])
                else:  # Use x component
                    origin = np.array([-plane[3] / plane[0], 0, 0])
                
                # Find two perpendicular vectors in the plane
                if abs(normal[2]) < 0.9:  # Normal not too close to Z
                    u = normalize_vector(cross_product(normal, np.array([0, 0, 1])))
                else:  # Normal close to Z, use X
                    u = normalize_vector(cross_product(normal, np.array([1, 0, 0])))

                v = cross_product(normal, u)
                
                # Create square corners
                corners = [
                    origin + plane_size * (-u - v),
                    origin + plane_size * (u - v),
                    origin + plane_size * (u + v),
                    origin + plane_size * (-u + v)
                ]
                
                # Add points
                point_ids = []
                for corner in corners:
                    points.InsertNextPoint(corner[0], corner[1], corner[2])
                    point_ids.append(points.GetNumberOfPoints() - 1)
                
                # Create polygon
                polygon = vtk.vtkPolygon()
                polygon.GetPointIds().SetNumberOfIds(4)
                for i, pid in enumerate(point_ids):
                    polygon.GetPointIds().SetId(i, pid)
                
                polys.InsertNextCell(polygon)
                plane_count += 1
        
        # Create polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(polys)
        
        # Write to file
        writer = vtk.vtkPolyDataWriter()
        writer.SetFileName(filename)
        writer.SetInputData(polydata)
        writer.Write()
        
        print(f"Exported {plane_count} cutting planes to {filename}")