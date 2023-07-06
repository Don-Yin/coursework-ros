from rich.traceback import install
from rich import print

install()

from src.modules.fcsv import FcsvParser
from src.utils.coordinate import point_to_numpy_idx
from src.utils.marching_cubes import marching_cubes
from pathlib import Path
import numpy as np
import torch
from functools import reduce
from SimpleITK import GetArrayFromImage, ReadImage
from random import sample
from itertools import product


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def image_to_rotated_array(itk_image):
    """
    Convert an ITK image to a numpy array with a 90 degrees rotation along the first and third axes.

    This function transforms an input ITK image into a numpy array. After converting, the array
    is rotated 90 degrees counter-clockwise in the plane made up of the first and third axes.

    Parameters:
        itk_image (itk.itkImagePython.itkImage): An image in ITK format.

    Returns:
        numpy.ndarray: The rotated numpy array corresponding to the input image.
    """
    array = GetArrayFromImage(itk_image)
    array = np.rot90(array, 1, axes=(0, 2))
    return array


def prepare_variables(files_dict: dict):
    # read the entries and targets ----------------------------
    entires_fcsv = FcsvParser(Path("data", "entries.fcsv"))
    targets_fcsv = FcsvParser(Path("data", "targets.fcsv"))

    entries_coords = torch.tensor(entires_fcsv.content_dataframe[["x", "y", "z"]].values, dtype=torch.float64).to(device)
    targets_coords = torch.tensor(targets_fcsv.content_dataframe[["x", "y", "z"]].values, dtype=torch.float64).to(device)

    # read the image ------------------------------------------
    images_itk = {k: [] for k in files_dict.keys()}
    samples_itk = {k: [] for k in files_dict.keys()}

    for key, value in files_dict.items():
        for file in value:
            images_itk[key].append(ReadImage(file))
        samples_itk[key].append(sample(images_itk[key], 1)[0])
        images_itk[key] = [image_to_rotated_array(i) for i in images_itk[key]]
        images_itk[key] = reduce(np.logical_or, images_itk[key])

    # prepare the meshes ---------------------------------------
    images_meshes = {k: {} for k in files_dict.keys()}

    for key, value in images_itk.items():
        verts, faces, _, _ = marching_cubes(value, 0.5)
        images_meshes[key]["verts"] = torch.tensor(verts.copy(), dtype=torch.float64).to(device)
        images_meshes[key]["faces"] = torch.tensor(faces.copy(), dtype=torch.float64).to(device)

    # check the images -----------------------------------------
    for value in samples_itk.values():
        for i in range(len(value) - 1):
            assert value[i].GetSize() == value[i + 1].GetSize(), "The images must have the same size."
            assert value[i].GetOrigin() == value[i + 1].GetOrigin(), "The images must have the same origin."
            assert value[i].GetDirection() == value[i + 1].GetDirection(), "The images must have the same direction."
            assert value[i].GetSpacing() == value[i + 1].GetSpacing(), "The images must have the same spacing."

    print("All images have the same size, origin, direction and spacing.")

    # convert the entries and targets to numpy indices ---------
    entries_coords_in_array = [point_to_numpy_idx(coord, list(samples_itk.values())[0][0]) for coord in entries_coords]
    targets_coords_in_array = [point_to_numpy_idx(coord, list(samples_itk.values())[0][0]) for coord in targets_coords]
    entries_coords_tensor = torch.stack(entries_coords_in_array)
    targets_coords_tensor = torch.stack(targets_coords_in_array)

    entries_targets_combs = list(product(entries_coords_tensor, targets_coords_tensor))
    return (
        entries_targets_combs,
        images_meshes,
        entries_coords,
        targets_coords,
        samples_itk,
        images_itk,
        entries_coords_in_array,
        targets_coords_in_array,
    )


files_dict_test = {
    "target_structure": [
        Path("data", "TestSet", "r_hippoTest.nii.gz"),
    ],
    "critical_structure": [
        Path("data", "TestSet", "ventriclesTest.nii.gz"),
        Path("data", "TestSet", "vesselsTestDilate1.nii.gz"),
    ],
    "cortex": [
        Path("data", "TestSet", "r_cortexTest.nii.gz"),
    ],
}

files_dict_real = {
    "target_structure": [
        Path("data", "BrainParcellation", "r_hippo.nii.gz"),
    ],
    "critical_structure": [
        Path("data", "BrainParcellation", "ventricles.nii.gz"),
        Path("data", "BrainParcellation", "vessels.nii.gz"),
    ],
    "cortex": [
        Path("data", "BrainParcellation", "cortex.nii.gz"),
    ],
}

# these variables are then imported in the main scripts
(
    entries_targets_combs,
    images_meshes,
    entries_coords,
    targets_coords,
    samples_itk,
    images_itk,
    entries_coords_in_array,
    targets_coords_in_array,
) = prepare_variables(files_dict_real)

if __name__ == "__main__":
    pass
