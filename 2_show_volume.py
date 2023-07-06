from src.variables import images_itk, entries_coords_in_array, targets_coords_in_array
from torch import tensor
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
import plotly.express as px
from src.utils.marching_cubes import marching_cubes


def show_volume(volume_array, entries_coords, targets_coords, valid_path=None):
    """
    Display a volume using matplotlib.

    Args:
        volume_array (numpy.ndarray): Array of the volume to be shown.
        entries_coords (list[tensor]): List of coordinates of the entries.
            Each coordinate can be a tensor, numpy array, or tuple representing a point in 3D space.
        targets_coords (list[tensor]): List of coordinates of the targets.
            Each coordinate can be a tensor, numpy array, or tuple representing a point in 3D space.
        valid_path (list[tensor, tensor], optional): Tuple of tensors representing the path(s) from entry to target.
            Each tensor can be an array or tuple of coordinates. This will be visualized as a line or lines.
            Defaults to None.

    Note:
        The volume will be displayed using a plotting library such as import plotly.graph_objects as go; import plotly.express as px.

    Example:
        show_volume(volume_array, entries_coords, targets_coords, valid_path)

    Procedures:
        - make a mesh using marching cubes
        - visualize the mesh using verts and faces
        - visualize the entries and targets using scatter3d
        - visualize the path using lines
    """
    verts, faces, _, _ = marching_cubes(volume_array, 0.5)

    x, y, z = zip(*verts)
    i, j, k = zip(*faces)

    mesh = go.Mesh3d(x=x, y=y, z=z, i=i, j=j, k=k, color="lightpink", opacity=0.5)

    fig = go.Figure(data=[mesh])

    # Convert entries and targets coords to plot-able format
    entries_x, entries_y, entries_z = zip(*[c.numpy() for c in entries_coords])
    targets_x, targets_y, targets_z = zip(*[c.numpy() for c in targets_coords])

    # Create Scatter3d object for entries
    scatter_entries = go.Scatter3d(
        x=entries_x,
        y=entries_y,
        z=entries_z,
        mode="markers",
        marker=dict(
            size=6,
            color="rgb(255, 0, 0)",  # red
        ),
        name="Entries",
    )

    # Create Scatter3d object for targets
    scatter_targets = go.Scatter3d(
        x=targets_x,
        y=targets_y,
        z=targets_z,
        mode="markers",
        marker=dict(
            size=6,
            color="rgb(0, 0, 255)",  # blue
        ),
        name="Targets",
    )

    # Add entries and targets to the figure
    fig.add_trace(scatter_entries)
    fig.add_trace(scatter_targets)

    # ------------------ #
    if valid_path is not None:
        for path in valid_path:
            path_start, path_end = [np.array(coord) for coord in path]
            path_start_x, path_start_y, path_start_z = path_start
            path_end_x, path_end_y, path_end_z = path_end

            # Create Scatter3d object for valid_path
            scatter_path = go.Scatter3d(
                x=[path_start_x, path_end_x],
                y=[path_start_y, path_end_y],
                z=[path_start_z, path_end_z],
                mode="lines",
                line=dict(color="rgb(0, 255, 0)", width=6),  # green
                name="Path",
            )

            # Add path to the figure
            fig.add_trace(scatter_path)

    fig.show()


if __name__ == "__main__":
    print(images_itk["target_structure"].shape)
    print(entries_coords_in_array.__len__())
    print(entries_coords_in_array[0])

    import json
    from pathlib import Path

    with open(Path("assets", "entry_target_real.json"), "r") as loader:
        valid_path = json.load(loader)["in_array_space"]
        valid_path = [valid_path]

    show_volume(
        images_itk["target_structure"],
        entries_coords_in_array,
        targets_coords_in_array,
        valid_path,
    )

    show_volume(
        images_itk["critical_structure"],
        entries_coords_in_array,
        targets_coords_in_array,
        valid_path,
    )

    show_volume(
        images_itk["cortex"],
        entries_coords_in_array,
        targets_coords_in_array,
        valid_path,
    )
