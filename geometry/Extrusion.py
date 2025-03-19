from geometry.Vector import Vector
from geometry.Polyline2D import Polyline2D, NamedPolyline, NamedCircle
from geometry.Polyline3D import Polyline3D
from geometry.Frame import Frame
from geo_functions import cross

class Extrusion(dict):
    """
    Represents a 3D extrusion created from a series of cross-sections positioned along a path.
    
    An Extrusion is created by placing 2D profiles (cross-sections) along a 3D path, each
    with its own orientation defined by a Frame. This class is fundamental for creating
    3D objects like pipes, wings, ship hulls, and other engineering structures.
    
    Inherits from dict to store the cross-sections with their associated names.
    """
    def __init__(self, name, list_of_3d_sections, list_of_frames, **kwargs):
        """
        Initialize an Extrusion object.
        
        Args:
            name (str): Name identifier for this extrusion
            list_of_3d_sections (list): List of 3D cross-sections (polylines) defining the shape
            list_of_frames (list): List of Frame objects defining the orientation at each section
            **kwargs: Optional parameters, such as holes for hollow extrusions
        
        Note:
            The number of sections must equal the number of frames.
            Each section is expected to have the same structure (same named polylines).
        """
        if len(list_of_3d_sections) == len(list_of_frames):
            # Store references to frames and sections
            self.list_of_frames = list_of_frames
            self.name = name
            self.ispipe = False
            _dico = {}
            
            # Get the first section as reference for the structure
            first_section = list_of_3d_sections[0]
            self.ordered_keys = []
            
            # Support for holes is currently commented out
            #if kwargs.has_key('holes'):
            #    self.holes = [k for k in kwargs['holes']]
            #    print(list_of_3d_sections[0])
            #    1/0
                
            # Initialize dictionary with the first section's components
            for key in first_section.ordered_keys:
                self.ordered_keys.append(self.name+'_'+key)
                _dico[self.name+'_'+key] = [first_section[key]]
                
            # Add remaining sections' components
            for other_sections in list_of_3d_sections[1:]:
                for key in first_section.ordered_keys:
                    _dico[self.name+'_'+key].append(other_sections[key])
                    
            # Store reference to all sections
            self.list_of_3d_sections = list_of_3d_sections
            
            # Debug output
            for key in _dico.keys():
                print(key)
                print(_dico[key])
                
            # Initialize the dict base class with our constructed dictionary
            super(Extrusion, self).__init__(**_dico)


    def write_topology(self):
        """
        Export the topology of this extrusion to a JSON file.
        
        The file will be named with the extrusion name and .topo extension.
        This method is useful for persisting the extrusion's structure.
        """
        import json
        json.dump(open(self.name+'.topo', 'w'), self)
        
    def classify_caps(self):
        """
        Identify and classify the start and end caps of the extrusion.
        
        This method analyzes the cross-sections at the beginning and end of the extrusion
        to determine which parts form the outer contour and which form interior holes.
        This is crucial for proper triangulation and mesh generation.
        
        Returns:
            dict: A dictionary containing classified contours and holes for both caps
                  Structure: {
                      'name_start': {'contour': [...], 'holes': [...]},
                      'name_end': {'contour': [...], 'holes': [...]}
                  }
        
        Note:
            Currently has a known issue with single section extrusions as noted in the code.
        """
        from ..geo_functions import classify_polylines

        # Initialize caps structure
        caps = {
                self.name+'_start': {'contour': [], 'holes': []},
                self.name+'_end': {'contour': [], 'holes': []}
                }
        
        # Get the frames for start and end caps
        frame_start = self.list_of_frames[0]
        frame_end = self.list_of_frames[-1]

        # Debug output
        print(self.list_of_3d_sections[0].keys())
        
        # Project section curves onto their respective frame planes
        sections_start = []
        sections_end = []
        for k in self.list_of_3d_sections[0].keys():
            sections_start.append(self.list_of_3d_sections[0][k].express_in_frame_plane(frame_start))
            sections_start.append(self.list_of_3d_sections[-1][k].express_in_frame_plane(frame_end))
        
        print(sections_start)

        # Classify start cap polylines as contours or holes
        incl, cont = classify_polylines(sections_start)
        
        # Debug output
        print('------- LIST OF 3D SECTIONS -------')
        print(self.list_of_3d_sections)
        print('-----------------------------------')
        print(type(self.list_of_3d_sections[0]))
        print('***********************************')
        
        # Get section keys for access
        the_keys = self.list_of_3d_sections[0].keys()
        
        # Populate start cap contours and holes
        for i, array in enumerate(cont):
            if array:
                caps[self.name+'_start']['contour'].append(self.list_of_3d_sections[the_keys[0]][i])
            else:
                caps[self.name+'_start']['holes'].append(self.list_of_3d_sections[the_keys[0]][i])
        
        # Classify end cap polylines as contours or holes
        incl, cont = classify_polylines(sections_end)
        for i, array in enumerate(cont):
            if array:
                caps[self.name+'_end']['contour'].append(self.list_of_3d_sections[-1][i])
            else:
                caps[self.name+'_end']['holes'].append(self.list_of_3d_sections[-1][i])

        # FIXME: Special case handling for single section extrusions
        # Current implementation has an issue where single sections are incorrectly classified as holes
        if len(sections_start) == 1:
            caps[self.name+'_start']['contour'] = [self.list_of_3d_sections[0][0]]
            caps[self.name+'_start']['holes'] = []
        if len(sections_end) == 1:
            caps[self.name+'_end']['contour'] = [self.list_of_3d_sections[-1][0]]
            caps[self.name+'_end']['holes'] = []
            
        return caps
        

    
    @property
    def array_of_closed_polylines(self):
        """
        Return an array of closed polylines from the extrusion sections.
        
        This property is deprecated and marked for deletion.
        
        Returns:
            list: Nested list of Polyline3D objects if the extrusion has no holes,
                  otherwise returns None
                  
        Note:
            This method is marked for removal in a future version.
        """
        if not self.holes:
            array_pol = []
            section_ordered_keys = self.list_of_3d_sections[0].ordered_keys
            
            # For each section, create a single polyline that combines all components
            for section in self.list_of_3d_sections:
                pol = []
                # Combine all points from all polylines in the section
                for k in section_ordered_keys:
                    for p in section[k]:
                        # Avoid duplicate points
                        if (not pol) or p != pol[-1]:
                            pol.append(p)
                # Create a 3D polyline from the points
                pol_3d = Polyline3D(pol)
                array_pol.append(pol_3d)
                
            return [array_pol]

    def add_to_ax(self, ax):
        """
        Visualize the extrusion on a matplotlib 3D axis.
        
        This method visualizes all polylines in the extrusion by delegating
        to their respective add_to_ax methods.
        
        Args:
            ax: Matplotlib 3D axis
        """
        for k in self.keys():
            for pol in self[k]:
                pol.add_to_ax(ax)  

    def triangulate(self):
        """
        Generate triangles representing the surface of the extrusion.
        
        This method creates triangles between adjacent cross-sections to form
        the surface of the extrusion. Each pair of corresponding line segments
        between consecutive cross-sections forms a quadrilateral, which is then
        triangulated.
        
        Returns:
            list: List of triangles representing the extrusion surface
        """
        tris = []
        
        # Process each named component of the extrusion
        for k in self.keys():
            # Process each cross-section except the last one
            for i, pol in enumerate(self[k][:-1]):
                # Create a polyline by joining the current section, the reversed next section, 
                # and closing back to the first point
                pol2 = Polyline3D(pol + self[k][i+1][::-1] + [pol[0]])
                # Alternative approach (commented out): pol2 = Polyline3D(pol + self[k][i+1][::-1])

                # Calculate tangent vector (along the section)
                tang1 = Vector([pol2[1][j] - pol2[0][j] for j in range(3)]).unit()
                
                # Calculate normal vector (perpendicular to the section)
                normal = Vector([pol2[len(pol)][j] - pol2[len(pol)-1][j] for j in range(3)]).unit()
                # Alternative approach (commented out): normal = cross(tang1, tang2)
                
                # Triangulate the polyline and add the resulting triangles
                tris += pol2.triangulate_first(normal)
                
        return tris

            



class Extrusion0(dict):
    def __init__(self,name, polyline, generator, frame_zero, n = 50, ax = None,*largs, **kwargs):
        """
         polyline is wether closed 2D polyline
         or a dict :
        
         {
         'name1' : [opened_polyline 1],
                 ...
         'nameN' : [opened_polyline N]
         }
         the reunion of all these polylines must be closed and have no zero-length segments
        
         generator must be an array of Frames

         along the generator, a parameter is designed by a real s in [0., 1.001]

         in kwargs we define FUNCTIONS for ex : 
          scale = lambda s: funcscale(s)
          rotation = lambda s: funcrotation(s)
          translate = lambda s: translate_rotation(s)
          ...
        """
        self.name = str(name)
        obj = {}
        frame_array = generator.frame_array_with_rmf(frame_zero, n=n)
        self.frames = frame_array
        self.ispipe = False

        if (type(polyline) == Polyline2D) and polyline.is_closed:
            obj['surface'] = []
            #print('closed polyline')
            for i,frame in enumerate(frame_array):
                s = float(i)/float(n-1)
                if kwargs.has_key('scale'):
                    scale = kwargs['scale'](s) 
                else: scale = 1.
                if kwargs.has_key('rotate'):
                    rotate = kwargs['rotate'](s) 
                else: rotate = 0.
                if kwargs.has_key('translate'):
                    translate = kwargs['translate'](s) 
                else: translate = [0., 0.]
                obj['surface'].append(polyline.to_frame(frame, scale = scale, translate = translate, rotate = rotate)) 
                if ax:
                    obj['surface'][-1].add_to_ax(ax)
            self.ordered_keys = ['surface']
        elif isinstance(polyline,NamedPolyline) :
            if (isinstance(polyline,NamedCircle)): self.ispipe = True
            self.ordered_keys = polyline.orderded_keys
            for k in polyline.keys():
                obj[self.name+'_'+str(k)] = []

            for i,frame in enumerate(frame_array):
                s = float(i)/max(1.,float(n-1))
                if kwargs.has_key('scale'):
                    scale = kwargs['scale'](s) 
                else: scale = 1.
                if kwargs.has_key('rotate'):
                    rotate = kwargs['rotate'](s) 
                else: rotate = 0.
                if kwargs.has_key('translate'):
                    translate = kwargs['translate'](s) 
                else: translate = [0., 0.]
                for k in polyline.keys():
                    named_polyline = polyline[k]
                    obj[self.name+'_'+str(k)].append(named_polyline.to_frame(frame, scale = scale, translate = translate, rotate = rotate))
                    if ax:
                        obj[self.name+'_'+str(k)][-1].add_to_ax(ax)
        super(Extrusion, self).__init__(obj)

    def frame_from_polyline(self, pol):
        for k in self:
            if pol in self[k]:
                return self.frames[self[k].index(pol)]

    def generator_tangent_at_polyline(self,pol):
        return self.frame_from_polyline(pol)[1]

    def interpolate_polylines(self):
        for k in self:
            for pol in self[k]:
                pol.spline_parameters()

    def filter_doublons(self):
        from ..geo_functions import assembly_polylines3d
        d = self
        points = []
        segments = []
        lines = {}
        ruled_facets = {}
        planar_caps = {'first': [], 'last': []}
        for k in d:
            lines[k] = []
            facets = []
            array_pol = d[k]
            for pol in array_pol:
                int_pol = []
                for p in pol:
                    if p not in points:
                        points.append(p)
                        int_pol.append(len(points)-1)
                    else:
                        int_pol.append(points.index(p))
                lines[k].append(int_pol)
            for icut, cut in enumerate(lines[k][:-1]):
                first = cut
                second = lines[k][icut+1]
                for iseg, seg in enumerate(first[:-1]):
                    facet = []

                    seg0 = [seg, first[iseg+1]]
                    seg1 = [first[iseg+1], second[iseg+1]]
                    seg2 = [second[iseg+1], second[iseg]]
                    seg3 = [second[iseg], seg ]
                   
                    for s in [seg0, seg1, seg2, seg3]:
                        if s in segments:
                            facet.append(('+',segments.index(s)))
                        elif s[::-1] in segments:
                            facet.append(('-',segments.index(s[::-1])))
                        else:
                            segments.append(s)
                            facet.append(('+',len(segments)-1))

                    facets.append(facet)

            ruled_facets[k] = facets
        first_cap_polylines = []
        last_cap_polylines = []
        for k in d.keys():
            first_cap_polylines.append(d[k][0])
            last_cap_polylines.append(d[k][-1])
        ordered_first = assembly_polylines3d(first_cap_polylines) 
        ordered_last = assembly_polylines3d(last_cap_polylines)     
        points_first_cap = [points.index(p) for p in ordered_first][::-1]
        points_last_cap = [points.index(p) for p in ordered_last]
        for ip,p0 in enumerate(points_first_cap[:-1]):
            p1 = points_first_cap[ip+1]
            s = [p0,p1]
            if s in segments:
                planar_caps['first'].append(('+',segments.index(s)))
            elif s[::-1] in segments:
                planar_caps['first'].append(('-',segments.index(s[::-1])))
            else:
                segments.append(s)
                planar_caps['first'].append(('+',len(segments)-1))
        for ip,p0 in enumerate(points_last_cap[:-1]):
            p1 = points_last_cap[ip+1]
            s = [p0,p1]
            if s in segments:
                planar_caps['last'].append(('+',segments.index(s)))
            elif s[::-1] in segments:
                planar_caps['last'].append(('-',segments.index(s[::-1])))
            else:
                segments.append(s)
                planar_caps['last'].append(('+',len(segments)-1))
        return points, ruled_facets, planar_caps, segments

    def add_to_geom(self,geom, do_caps = True, reverse = False, lcar = 0.5):
        points, facets, caps, seg = self.filter_doublons()
        gmsh_points = []
        gmsh_lines = []
        sfloop = []
        features = []
        for p in points:
            gmsh_points.append(geom.add_point(p , lcar))
        for s in seg:
            gmsh_lines.append(geom.add_line(gmsh_points[s[0]], gmsh_points[s[1]]))
        for key in facets:
            phys = []
            array_seg = facets[key]
            for loop in array_seg:
                ll = []
                for s in loop:
                    if reverse:
                        if s[0] == '+':ori = '-'
                        if s[0] == '-':ori = '+'
                    else:ori = s[0]
                    if ori == '+':
                        ll.append(gmsh_lines[s[1]])
                    if ori == '-':
                        ll.append(-gmsh_lines[s[1]])
                    features.append(ll[-1])
                lloop = geom.add_line_loop(ll)
                sf = geom.add_surface(lloop)
                phys.append(sf)
                sfloop.append(sf) 
            geom.add_physical_surface(phys,key)
        
        if do_caps:
            for key in caps:
                phys = []
                array_seg = caps[key]
                ll = []
                for s in array_seg:
                    if s[0] == '+':
                        ll.append(gmsh_lines[s[1]])
                    if s[0] == '-':
                        ll.append(-gmsh_lines[s[1]])
                lloop = geom.add_line_loop(ll)
                sf = geom.add_plane_surface(lloop)
                phys.append(sf)
                sfloop.append(sf) 
                geom.add_physical_surface(phys,key)
            volume = geom.add_volume(geom.add_surface_loop(sfloop))
        return features

                    
    def triangulate(self):
        tris = []
        for k in self.keys():
            for i,pol in enumerate(self[k][:-1]):
                pol2 = Polyline3D(pol + self[k][i+1][::-1] + [pol[0]])
                #pol2 = Polyline3D(pol + self[k][i+1][::-1] )

                tang1 = Vector([ pol2[1][j] - pol2[0][j] for j in range(3)]).unit()
                normal = Vector([ pol2[len(pol)][j] - pol2[len(pol)-1][j] for j in range(3)]).unit()
                #normal = cross(tang1, tang2)
                tris+=pol2.triangulate_first(normal)
        return tris


    def pop_to_geom(self, geom):
        for k in self.keys():
            for pol3d in self[k]:
                pol3d.pop_to_geom(geom)
