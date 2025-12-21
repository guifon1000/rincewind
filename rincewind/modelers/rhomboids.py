import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import meshio 
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class RhomboidShape:
    def __init__(self, radius, N, triangulated):
        self.vertices = generate_planar_rhombus_points(N, radius, 1, N-1)
        self.rhombus = []
        for i in range(len(self.vertices)//4):
            self.rhombus.append([4*i, 4*i+1, 4*i+2, 4*i+3])
        self.triangulated = triangulated

        self.unique_points()


    def unique_points(self):
        unique = []
        cells = []
        self.rhombus = []
        for i,v in enumerate(self.vertices):
            
            if v not in unique:
                unique.append(v)
            cells.append(unique.index(v))
            if i%4==3:
                print(cells)
                self.rhombus.append(cells)
                cells = []
        self.vertices = unique
        #return unique
    


    def create_mesh(self):
        if self.triangulated:
            triangles = []
            for quad in self.rhombus:
                triangles.append([quad[0], quad[1], quad[3]])
                triangles.append([quad[1], quad[2], quad[3]])
            self.mesh = meshio.Mesh(
                points=np.array(self.vertices),
                cells=[("triangle", np.array(triangles))],
            )
        else:
            self.mesh = meshio.Mesh(
                points=np.array(self.vertices),
                cells=[("quad", np.array(self.rhombus))],
            )

    def show_matplotlib(self):
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        plt.axis('equal')
        points_array = np.array(self.mesh.points)
        #for p in shape.mesh.points:
            #ax.plot([p[0], 0.], [p[1], 0.], [p[2], 0.], 'r')
        # Create collection of quad faces
        faces = []
        vec = []
        if self.triangulated: 
            vec = self.mesh.cells_dict['triangle']
        else:
            vec = self.mesh.cells_dict['quad']
        for cell in vec:
            print(cell)
            quad_points = self.mesh.points[cell]
            faces.append(quad_points)



        # Plot filled quads
        poly_collection = Poly3DCollection(faces, alpha=0.4, facecolors='lightblue', edgecolors='blue')
        ax.add_collection3d(poly_collection)

        # Set axis limits
        ax.set_xlim([points_array[:, 0].min(), points_array[:, 0].max()])
        ax.set_ylim([points_array[:, 1].min(), points_array[:, 1].max()])
        ax.set_zlim([points_array[:, 2].min(), points_array[:, 2].max()])
        ax.limgs = 'equal'
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Planar Rhombus Crown Geometry')
        plt.show()

def generate_next_planar_rhombus_crown(vecPnm1, vecPn):
    N = len(vecPn)
    vecPnp1 = []
    for k in range(N):
        k_next = (k + 1) % N

        v1 = [vecPn[k][i] - vecPnm1[k_next][i] for i in range(3)]
        v2 = [vecPn[k_next][i] - vecPnm1[k_next][i] for i in range(3)]
        v3 = [(v1[i] + v2[i]) for i in range(3)]
        #print(v1, v2, v3)
        orig =  [vecPnm1[k_next][i] for i in range(3)]
        for v in [[0,0,0], v1, v3, v2]:
            p = [orig[i] + v[i] for i in range(3)]
            vecPnp1.append(p)
            
        #vecPnp1.append(vecPnm1[k] + v3)
    return vecPnp1

def generate_planar_rhombus_points(N, r, h, M):
    pointsm1 = [[0,0,0] for _ in range(N)]
    points_next = [[r * np.cos(2*np.pi*k/N), r * np.sin(2*np.pi*k/N), h] for k in range(N)]
    points = []
    for i in range(M):
        crown_points = generate_next_planar_rhombus_crown(pointsm1, points_next)
        points += crown_points
        pointsm1 = points_next
        # Extract every 4th point starting from index 2 (the outer vertices of each rhombus)
        points_next = [crown_points[4*k + 2] for k in range(N)]
    return points


  
# Parameters
N = 8  # Number of points

r = 0.75  # Radius
h = 3.0   # Height
M = N-1  # Number of layers

# Generate points

shape = RhomboidShape(r, N, triangulated=False)
#shape.create_mesh()
#shape.mesh.write("planar_rhombus_000.vtk")
#shape.show_matplotlib()
