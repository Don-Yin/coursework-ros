import numpy as np
from torch import tensor
import torch


def point_to_numpy_idx(coord: tuple, itk_image, round: bool = False):
    """Convert a point coordinate to a numpy index.
    Args:
        coord: Tuple of 3D point coordinate.
        itk_image: ITK image object.
        round: Boolean indicating whether to round the result to the nearest integer. Defaults to False.

    Returns:
        The numpy index corresponding to the input point coordinate.
    """
    shape = itk_image.GetSize()

    # (point - offset) * inverse of the (direction matrix * spacing matrix) = index
    offset = itk_image.GetOrigin()
    direction = itk_image.GetDirection()
    spacing = itk_image.GetSpacing()

    coord = coord.cpu().numpy()  # convert to numpy array

    idx = np.subtract(coord, offset)  # (point - offset)

    direction = np.reshape(direction, (3, 3))  # reshape
    spacing = np.reshape(spacing, (1, 3))  # reshape

    product = [
        direction[0] * spacing[0][0],
        direction[1] * spacing[0][1],
        direction[2] * spacing[0][2],
    ]  # a bit strange but works
    product = np.array(product)  # convert to numpy array

    product = np.linalg.inv(product)  # inverse
    idx = np.matmul(product, idx)  # multiply
    idx = np.round(idx).astype(int) if round else idx  # round and convert to int

    # reverse negative indices
    for i in range(len(idx)):
        if idx[i] < 0:
            idx[i] += shape[i]

    idx = tensor(idx, dtype=torch.float64).to(torch.device("cuda" if torch.cuda.is_available() else "cpu"))  # convert to tensor
    return idx
