import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from skimage import measure

import warnings

# Filter out warnings with a 'UserWarning' category
warnings.filterwarnings("ignore", category=UserWarning)


def marching_cubes(volume: np.ndarray, level: float = 0.5, visualize=False, lines: list = None):
    # Use marching cubes to obtain the surface mesh of these ellipsoids
    verts, faces, normals, values = measure.marching_cubes(volume, level=level)

    if visualize:
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection="3d")

        # Fancy indexing: `verts[faces]` to generate a collection of triangles
        mesh = Poly3DCollection(verts[faces])
        mesh.set_edgecolor("k")
        ax.add_collection3d(mesh)

        if lines is not None:
            for line in lines:
                ax.plot(
                    [line[0][0], line[1][0]],
                    [line[0][1], line[1][1]],
                    [line[0][2], line[1][2]],
                    "r-",
                )

        ax.set_xlabel("x-axis:")
        ax.set_ylabel("y-axis:")
        ax.set_zlabel("z-axis:")

        # set the limits of the plot to the limits of the data
        ax.set_xlim(0, volume.shape[0])
        ax.set_ylim(0, volume.shape[1])
        ax.set_zlim(0, volume.shape[2])

        plt.tight_layout()
        plt.show()

    return verts, faces, normals, values
