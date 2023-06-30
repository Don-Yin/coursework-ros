from src.variables import images_meshes
import numpy as np
from stl import mesh

print(images_meshes[0])

# # Define the 8 vertices of the cube
# vertices = np.array(verts)
# # Define the 12 triangles composing the cube
# faces = np.array(faces)

# # Create the mesh
# cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))

# # In the "cube" mesh, assign the vertices and vectors from verts and faces
# for i, f in enumerate(faces):
#     for j in range(3):
#         cube.vectors[i][j] = vertices[f[j], :]

# # Write the mesh to file "cube.stl"
# cube.save("cube.stl")
