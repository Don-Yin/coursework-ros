I now have some vertices and faces from scikit image marching_cubes 

verts, faces, _, _ = measure.marching_cubes(volume, 0.5)

how to convert this into stl format and save to stl files

----

To save your mesh in STL format, you can use the numpy-stl library. Here's a sample code:

```python
import numpy as np
from stl import mesh

# Define the 8 vertices of the cube
vertices = np.array(verts)
# Define the 12 triangles composing the cube
faces = np.array(faces)

# Create the mesh
cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))

# In the "cube" mesh, assign the vertices and vectors from verts and faces
for i, f in enumerate(faces):
    for j in range(3):
        cube.vectors[i][j] = vertices[f[j], :]

# Write the mesh to file "cube.stl"
cube.save('cube.stl')
```
In this code:

- `mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))` creates a new mesh with the same number of triangles as your original mesh.
- `for i, f in enumerate(faces):` iterates through each face in the list of faces.
- `for j in range(3):` iterates through each vertex in the current face.
- `cube.vectors[i][j] = vertices[f[j], :]` assigns the x, y, z coordinates of each vertex in the current face to the corresponding vertex in the cube mesh.
- `cube.save('cube.stl')` saves the cube mesh to a file named 'cube.stl'. You can replace 'cube.stl' with the name you want for your STL file.

Before running the code, make sure you have the numpy-stl library installed. You can install it using pip:

```bash
pip install numpy-stl
```

Note: The numpy-stl library expects faces to be composed of 3 vertices. If your faces have more than 3 vertices, you'll need to convert them into triangles before using this code.