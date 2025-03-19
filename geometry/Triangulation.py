import sys
sys.path.append('../')
from geo_functions import dot
import json
import numpy as np
from geometry.Point import Point
from geometry.Vector import Vector
from geometry.Triangle import Triangle
from scipy.spatial.kdtree import *

class Triangulation(dict):
    """
    DO WE COMPUTE THE INDICES FROM 0 OR 1 ???
    dictionary
    * KEYS : 
        'vertices' : array of points, 3-real arrays (x,y,z)
        'faces'    : array of faces, 3 int arrays (3 index in vertices list)
    * TODO !!!! 
        PHYSICAL GROUPS :
        'faces' becomes a dict :
        key = 'physical_group' :
               ((i, j, k), ..., (l, m, n)) (the faces belonging to 'physical groups')
        if None specified:
        key = 'default' :
               ( all the faces )
        problem : how to specify this correctly to init:
                  test if dict / array ?

    """
    def __init__(self, vertices = None, faces = None, **kwargs):
        
        print(type(faces))
        if vertices : 
            self['vertices'] = []
            for v in vertices:
                self['vertices'].append(v) 
            self.vertices = self['vertices']
        faces_2 = {}
        if ('physical' in kwargs) and ('belongs' in kwargs) :
            self['physical'] = kwargs['physical']
            self['belongs'] = kwargs['belongs']
            for p in self['physical']:
                faces_2[p[1]] = []
                for i,f in enumerate(faces):
                    if self['belongs'][i] == p[0]:
                        faces_2[p[1]].append(i)
        elif type(faces) is dict:
            faces_2 = faces
        elif type(faces) is list:
            faces_2['default'] = faces
            print("zob")
            print(faces)
        elif type(faces) is np.ndarray:
            faces_2['default'] = [f for f in faces]
        self['faces'] = faces_2
        if 'edges' in kwargs:
            self['edges'] = kwargs['edges']
        else:
            self['edges'] = None

    @property
    def cg(self):
        return np.mean(np.array(self['vertices'])   ,axis=0)

    @property 
    def bounding_box(self):
        bounds = [
                 min([p[0] for p in self['vertices']]),
                 min([p[1] for p in self['vertices']]),
                 min([p[2] for p in self['vertices']]),
                 max([p[0] for p in self['vertices']]),
                 max([p[1] for p in self['vertices']]),
                 max([p[2] for p in self['vertices']])]
        return bounds


    def computeKDTree(self):
        vert = []
        self.kdtree = KDTree(self.vertices)
        print(self.kdtree)

    def neighbours(self, triangle):
        if triangle in self.triangle_list:
            neig = [None,None,None]
            for i,t in enumerate(self.triangle_list):
                #if t.any() != tri.any():
                    if ((triangle[0] in t) and (triangle[1] in t) and (triangle[2] not in t)) : neig[0]=t
                    elif ((triangle[1] in t) and (triangle[2] in t) and (triangle[0] not in t)) : neig[1]=t
                    elif ((triangle[2] in t) and (triangle[0] in t) and (triangle[1] not in t)) : neig[2]=t
            print(neig)
            return neig


    def translate(self, vec):
        new_pts = [Point([p[i] + vec[i] for i in range(3)]) for p in self['vertices']]
        self['vertices'] = new_pts

    def write_obj_file(self,name):
        vn = self.vertex_normals()
        print(self.keys())
        if '.obj' in name :
            with open(name, 'w') as f:
                f.write('# OBJ file\n')
                f.write('o '+str(name)+'\n')
                for v in self['vertices']:
                    f.write('v %.4f %.4f %.4f\n' % (v[0],v[1],v[2]))
                for v in vn:
                    f.write('vn %.4f %.4f %.4f\n' % (v[0],v[1],v[2]))
                for group in self['faces']:
                    for i,t in enumerate(self['faces'][group]):
                        f.write('f')
                        for p in t :
                            f.write(" %d//%d" % (p, p))
                        f.write('\n')

        elif '.json' in name :
            f = open(name, 'w')
            json.dump(self._dict(), f, indent =4)

    def write_stl_file(self, name, separate_surfaces=True):
        f = open(name+('.stl'), 'w')
        if separate_surfaces==True:
            for face in self['faces']:
                f.write('solid '+face+'\n')
                for tri in self['faces'][face]:
                    tri_array = []
                    for pt_idx in tri:
                        tri_array.append(self['vertices'][pt_idx-1])
                    triangle = Triangle(tri_array)
                    st = 'facet normal '
                    for i in range(3):
                        st+=str(triangle.normal[i])+' '
                    st+='\n'
                    f.write(st)
                    f.write('outer loop\n')
                    for pt in triangle :
                        st = 'vertex '
                        for coord in pt:
                            st+=str(coord)+' '
                        f.write(st+'\n')
                    f.write('endloop\n')
                    f.write('endfacet\n')
                f.write('endsolid '+face+'\n')
        else:
            f.write('solid '+name+'\n')
            for face in self['faces']:
                for tri in self['faces'][face]:
                    tri_array = []
                    for pt_idx in tri:
                        tri_array.append(self['vertices'][pt_idx-1])
                    triangle = Triangle(tri_array)
                    st = 'facet normal '
                    for i in range(3):
                        st+=str(triangle.normal[i])+' '
                    st+='\n'
                    f.write(st)
                    f.write('outer loop\n')
                    for pt in triangle :
                        st = 'vertex '
                        for coord in pt:
                            st+=str(coord)+' '
                        f.write(st+'\n')
                    f.write('endloop\n')
                    f.write('endfacet\n')
            f.write('endsolid '+name+'\n')
        



    def write_fms_file(self, name, patches=[], symmetry_planes=[]):
        f = open(name+('.fms'),'w')
        print('------------------------------------------')
        print('------------------ FMS PART JHJ--------------')
        print('------------------------------------------')
        f.write('// patch names and types\n')
        f.write(str(len(self['faces']))+'\n')
        f.write('(\n')
        for p in self['faces']:
            f.write(str(p)+'\n')
            if p in patches:
                f.write('patch\n')
            elif p in symmetry_planes:
                f.write('symetryPlane\n')
            else:
                f.write('wall\n')
            f.write('\n')
        f.write(')\n\n')
        f.write('// coordinates of surface points\n')
        f.write(str(len(self['vertices']))+'\n')
        f.write('(\n')
        for p in self['vertices']:
            f.write( '( %f %f %f) '%(p[0], p[1], p[2] )   )
        f.write(')\n\n')
        f.write('// list of triangles\n')
        ntri = 0
        for k in self['faces']:
            ntri += len(self['faces'][k])
        f.write(str(ntri)+'\n')
        f.write('(\n')
        for i,group in enumerate(self['faces']):
            for j,t in enumerate(self['faces'][group]):
                #tp = '( ( %d %d %d )  %d ) '% (t[0] - 1 , t[1] - 1 , t[2] - 1 , i   )
                tp = '( ( %d %d %d )  %d ) '% (t[0]  , t[1]  , t[2]  , i   )
                f.write(tp)
        f.write(')\n\n')
        print(self['edges'])
        if self['edges']:
            f.write(str(len(self['edges']))+'(')
            f.write('\n')
            for s in self['edges']:
                ed = '( %d %d  )\n'%(s[0] , s[1])
                f.write(ed)
            f.write(')')
        else:
            f.write('0()\n')
        for i in range(3):f.write('0()\n')  

    @property
    def triangle_list(self):
        _triangles = []
        for k in self['faces'].keys():
            for f in self['faces'][k]:
                p0 = Point(self['vertices'][f[0]-1])
                p1 = Point(self['vertices'][f[1]-1])
                p2 = Point(self['vertices'][f[2]-1])
                tr = Triangle((p0, p1, p2))
                _triangles.append(tr)
        return _triangles
    
     
    def reorient_convex(self):
        # reorient the faces IF CONVEX
        _new_faces = {}
        for group in self['faces']:
            _new_faces[group] = []
            for f in self['faces'][group] :
                p0 = Point(self['vertices'][f[0]-1])
                p1 = Point(self['vertices'][f[1]-1])
                p2 = Point(self['vertices'][f[2]-1])
                tr = Triangle((p0, p1, p2))
                n0 = tr.normal.unit()
                n1 = Vector(tr.cg)
                if dot(n0,n1) < 0:
                    fi = (f[0], f[2], f[1])
                else:
                    fi = f
                _new_faces[group].append(fi)
        self['faces'] = _new_faces
        # ok the convex is correctly oriented in the dict d

    def translate(self, vec):
        _vertices = []
        for p in self['vertices'] :
            _vertices.append([\
                    p[0] + vec[0],\
                    p[1] + vec[1],\
                    p[2] + vec[2]])
        self['vertices'] = _vertices

    def section_by_plane(self,plane):
        from rincewind.geo_functions import is_on_plane, cut_triangle_by_plane 
        """
        Warning : only a point cloud will be given
        """
        cloud = []
        for t in self.triangle_list:
            tris = cut_triangle_by_plane(t,plane)
            if tris:
                for ti in tris:
                    #cloud.append(ti)
                    for p in ti:
                        if is_on_plane(p,plane, 1.e-12):
                            cloud.append(p)
        return cloud


    def reverse(self):
        _faces = {}
        for group in self['faces']:
            _faces[group] = []
            for f in self['faces'][group]:
                _faces[group].append([f[0], f[2], f[1]])
        return Triangulation(vertices = self['vertices'],\
                             faces = _faces, edges = self['edges'])
    def get_triangles(self):
        _triangles = []
        for group in self['faces']:
            for f in self['faces'][group]:
                p = []
                p.append(self['vertices'][f[0]-1])
                p.append(self['vertices'][f[1]-1])
                p.append(self['vertices'][f[2]-1])
                _triangles.append(Triangle(p))
        return _triangles

    def vertex_normals0(self):
        triangles = self.get_triangles()
        vertex_normals = []
        for i,p in enumerate(self['vertices']):
            _faces = []
            for group in self['faces']:
                for j,f in enumerate(self['faces'][group]):
                    if (i+1) in f:
                        _faces.append(triangles[j])
            vertex_normal = [0., 0., 0.]
            for j,f in enumerate(_faces):
                vertex_normal = [vertex_normal[k] + f.normal[k] for k in range(3) ]
            vertex_normal = Vector(vertex_normal)
            vertex_normal = vertex_normal.unit()
            vertex_normals.append(vertex_normal)
        return vertex_normals
    
    def vertex_normals(self):
        vertex_normals = [[] for p in self['vertices']]
        for group in self['faces']:
            for j,f in enumerate(self['faces'][group]):
                vertex_1 = self['vertices'][f[0]-1]
                vertex_2 = self['vertices'][f[1]-1]
                vertex_3 = self['vertices'][f[2]-1]
                nrm = Triangle([vertex_1, vertex_2, vertex_3]).normal
                vertex_normals[f[0]-1].append(nrm)
                vertex_normals[f[1]-1].append(nrm)
                vertex_normals[f[2]-1].append(nrm)
        vn = []
        import numpy as np
        for veclist in vertex_normals:
            vec = Vector([np.mean([vec[0] for vec in veclist]), np.mean([vec[1] for vec in veclist]), np.mean([vec[2] for vec in veclist])]).unit()
            vn.append(vec)
        return vn

    def refine_2(self) :
        d2 = {}
        _vertices = []
        _faces = {}
          
        print(self['faces'])
        for group in self['faces']:
            _faces[group] = []
            for f in self['faces'][group]:
                p0 = self['vertices'][f[0]-1]
                p1 = self['vertices'][f[1]-1]
                p2 = self['vertices'][f[2]-1]
                if p0 in _vertices:
                    i0 = _vertices.index(p0)
                else:
                    _vertices.append(p0)
                    i0 = len(_vertices)-1
                if p1 in _vertices:
                    i1 = _vertices.index(p1)
                else:
                    _vertices.append(p1)
                    i1 = len(_vertices)-1
                if p2 in _vertices:
                    i2 = _vertices.index(p2)
                else:
                    _vertices.append(p2)
                    i2 = len(_vertices)-1

                p3 = Point([0.5*(p1[j]+p2[j]) for j in range(3)])
                p4 = Point([0.5*(p2[j]+p0[j]) for j in range(3)])
                p5 = Point([0.5*(p1[j]+p0[j]) for j in range(3)])

                if p3 in _vertices:
                    i3 = _vertices.index(p3)
                else:
                    _vertices.append(p3)
                    i3 = len(_vertices)-1
                if p4 in _vertices:
                    i4 = _vertices.index(p4)
                else:
                    _vertices.append(p4)
                    i4 = len(_vertices)-1
                if p5 in _vertices:
                    i5 = _vertices.index(p5)
                else:
                    _vertices.append(p5)
                    i5 = len(_vertices)-1
                i0 += 1 
                i1 += 1 
                i2 += 1 
                i3 += 1 
                i4 += 1 
                i5 += 1
                _faces[group].append((i4, i3, i2))
                _faces[group].append((i5, i1, i3))
                _faces[group].append((i5, i3, i4))
                _faces[group].append((i0, i5, i4))
            _vertices2 = []
        return Triangulation(vertices = _vertices, faces = _faces)


 
def read_msh_triangulation(fi, **kwargs):
    f = open(fi,'r').readlines()
    _vertices = []
    app = False
    ist = 0
    nv = None
    while app == False :
        l = f[ist] 
        if l.startswith('$Nodes'):
            nv = int(f[ist+1])
            app = True
        ist+=1
    for i in range(ist+1,ist+nv+1):
        l=f[i].split()
        _vertices.append((float(l[1]), float(l[2]), float(l[3])))
    _faces = []
    for i in range(ist+nv+1,len(f)):
        lp = f[i].split()
        if (len(lp) > 3) and (lp[1] == '2') :
                _faces.append((int(lp[5]),\
                                  int(lp[6]),\
                                  int(lp[7])))
    return Triangulation(_vertices, _faces)

def merge(tri_1, tri_2):
    # tri_1 is chosen to be kept identical, tri_2 will be adapted
    n_physical_1 = len(tri_1['faces'])
    print('tri 1 has '+str(n_physical_1)+' physical groups of faces')
    n_pt_1 = len(tri_1['vertices'])
    n_physical_2 = len(tri_2['faces'])
    print('tri 2 has '+str(n_physical_2)+' physical groups of faces')
    n_pt_2 = len(tri_2['vertices'])
    _faces = tri_1['faces']
    # the physical groups of tri_2 are offset by n_physical_1
    # the index of the points are offset by n_pt_1
     
    _vertices = tri_1['vertices'] + tri_2['vertices']
    for group in tri_2['faces']:
        print(group)
        _faces[group] = []
        for f in tri_2['faces'][group]:
            new_triangle = [f[0] + n_pt_1 , f[1] + n_pt_1 , f[2] + n_pt_1 ]
            _faces[group].append(new_triangle)

    return Triangulation(vertices = _vertices , faces = _faces)


def read_json_file(name):
    f = json.load(open(name,'r'))
    try:
        return Triangulation(vertices=f['vertices'], faces=f['faces'])
    except:
        print('not enough info in '+name)
        return 0




def read_stl_file(name, **kwargs):
    fl = open(name+'.stl', 'r').readlines()
    dico = {}
    dico['vertices'] = []
    dico['faces'] = {}
    dico['faces']['default'] = []
    istart = None
    iend = None
    solid_name = None
    for i,line in enumerate(fl):
        if line.startswith('solid'):
            istart = i
            solid_name = line.split()[1]
            for j in range(i, len(fl)):
                linej = fl[j]
                if linej.startswith('endsolid') and linej.split()[1]==solid_name:
                    iend = j
        if istart and iend and solid_name:
            break
    for i in range(istart+1, iend-1):
        li = fl[i]
        if 'facet normal' in li:
            vn = Vector([float(li.split()[k+2]) for k in range(3)])
            vertices = []
            for j in range(i+2, i+5):
                pj = Point([float(fl[j].split()[k+1]) for k in range(3)])
                vertices.append(pj)
            face = []
            for pt in vertices:
                if not pt in dico['vertices']:
                    dico['vertices'].append(pt)
                    ipt = len(dico['vertices']) - 1
                else:
                    ipt = dico['vertices'].index(pt)
                face.append(ipt+1)
            dico['faces']['default'].append(face)
    return Triangulation(**dico)



        


 
def read_msh_file(name,**kwargs):
    #print('Reading MSH file')
    d = {}
    lmsh = open(name+'.msh','r').readlines()
    d['faces'] = {}
    physical_correspondance = {} 
    for i,l in enumerate(lmsh) :
        if '$PhysicalNames' in l:
            N = int(lmsh[i+1])
            for j in range(N):
                il = i+j+2
                lpn = lmsh[il].split()
                #print(lpn)
                number = int(lpn[1])
                if int(lpn[0])==2:
                    name = str(lpn[2].replace('\"',''))
                    d['faces'][name] = []
                    physical_correspondance[name] = number
                elif int(lpn[0])==1 and (not 'edges' in d):
                    d['edges']=[]
                    #print(lpn)
                    #print('************')
            i = i+N+2
            break
    i0 = i
    #print(physical_correspondance)
    for i,l in enumerate(lmsh) :
        if '$Nodes' in l:
            d['vertices']=[]
            N = int(lmsh[i+1])
            for j in range(N):
                il = i+j+2
                lpn = lmsh[il].split()
                pt = Point(   [float(lpn[1]),float(lpn[2]),float(lpn[3])]    )
                d['vertices'].append(pt)
            i = i+N+2
            break
    i0 = i
    flog = open('logFMS','w')
    for i,l in enumerate(lmsh) :
        if '$Elements' in l:
            N = int(lmsh[i+1])
            for j in range(N):
                il = i+j+2
                lpn = lmsh[il].split()
                if int(lpn[1])==2:
                    flog.write('triangle :'+str(int(lpn[0]))+'\n')
                    tags = int(lpn[2])
                    phys = int(lpn[1+tags])
                    flog.write('physical :'+str(phys)+'\n')
                    p0 = int(lpn[2+tags+1]) + 1
                    p1 = int(lpn[2+tags+2]) + 1
                    p2 = int(lpn[2+tags+3]) + 1
                    for group in d['faces']:
                        if phys==physical_correspondance[group]:
                            flog.write('triangle belongs to '+group+'\n')
                            # why is this triangle written reversed ?
                            tri = (p2-1,p1-1,p0-1)
                            #pt = ((p2-1,p1-1,p0-1),k[0]-1)
                            d['faces'][group].append(Triangle(tri))
                            break
                elif int(lpn[1])==1:
                    #print(lpn)
                    #1/0
                    
                    if d.has_key('edges'):
                        p1 = int(lpn[-1])
                        p2 = int(lpn[len(lpn)-2])
                        d['edges'].append([p1-1, p2-1])

            i = i+N+2
            break
    i0 = i
    #print('done')
    return Triangulation(**d) 







 
