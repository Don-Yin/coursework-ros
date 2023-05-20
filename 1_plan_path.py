from rich import print

from src.variables import entries_targets_combs, images_meshes, device, entries_coords, targets_coords, samples_itk
import torch
from torch import tensor
from tqdm import tqdm
import numpy as np
from src.utils.intersect import check_intersect, check_intersect_batch
from src.utils.coordinate import point_to_numpy_idx
import json

entires = [i[0] for i in entries_targets_combs]
targets = [i[1] for i in entries_targets_combs]
entires = torch.stack(entires)
targets = torch.stack(targets)


def is_valid_index(
    intersect_target,
    intersect_critical,
    intersect_cortex,
    # -----------------------
    length_target,
    length_critical,
    length_cortex,
    # -----------------------
    distance_target,
    distance_critical,
    distance_cortex,
    # -----------------------
    angle_target,
    angle_critical,
    angle_cortex,
):
    """
    Check whether an index is valid based on intersection conditions and other parameters.
    Returns:
    bool: Whether the index is valid.
    """
    # Add or modify conditions as necessary
    condition = intersect_target and not intersect_critical  # existing condition
    # trajectory is below "a certain length" where the length is arbitrary
    # note that this is the distance in numpy coordinates, not physical coordinates
    condition = condition and (length_target < 50)
    # if intersect_cortex:
    #     condition = condition and (angle_cortex < (90 - 55))

    return condition


def find_max_distance_and_indices(entries, targets, images_meshes, batch_size: int = 1000):
    """
    Break entries_targets_combs into batches of size batch_size.
    For each batch, check the intersection and get the valid indices in the original entries_targets_combs.
    Among the valid indices, find the entry and target with the maximum distance from the critical structure.

    Parameters:
    entries (torch.Tensor): Tensor of entries.
    targets (torch.Tensor): Tensor of targets.
    images_meshes (dict): Dictionary containing target and critical structures.
    batch_size (int, optional): The size of each batch. Default is 1000.

    Returns:
    tuple: The entry and target with the maximum distance from the critical structure and the list of valid indices.
    """

    max_distance = 0
    max_distance_entry = None
    max_distance_target = None
    valid_indices = []

    # Calculate the number of batches
    num_batches = (entries.shape[0] + batch_size - 1) // batch_size

    for i in tqdm(range(num_batches), desc="Processing batches"):
        # Get the start and end indices for the current batch
        start_idx = i * batch_size
        end_idx = min((i + 1) * batch_size, entries.shape[0])

        # Slice the entries and targets for the current batch
        entries_batch = entries[start_idx:end_idx]
        targets_batch = targets[start_idx:end_idx]

        # Perform the intersection check for the current batch
        intersects_target, lengths_target, distances_target, angles_target = check_intersect_batch(
            entries_batch, targets_batch, **images_meshes["target_structure"]
        )
        intersects_critical, lengths_critical, distances_critical, angles_critical = check_intersect_batch(
            targets_batch, entries_batch, **images_meshes["critical_structure"]
        )
        intersects_cortex, lengths_cortex, distances_cortex, angles_cortex = check_intersect_batch(
            entries_batch, entries_batch, **images_meshes["cortex"]
        )

        # Get the valid indices for the current batch
        valid_indices_batch = [
            i
            for i in range(start_idx, end_idx)
            if is_valid_index(
                intersects_target[i - start_idx],
                intersects_critical[i - start_idx],
                intersects_cortex[i - start_idx],
                # --------------------------
                lengths_target[i - start_idx],
                lengths_critical[i - start_idx],
                lengths_cortex[i - start_idx],
                # --------------------------
                distances_target[i - start_idx],
                distances_critical[i - start_idx],
                distances_cortex[i - start_idx],
                # --------------------------
                angles_target[i - start_idx],
                angles_critical[i - start_idx],
                angles_cortex[i - start_idx],
            )
        ]

        # For each valid index in the batch, check if the distance from the critical structure is greater than the current max distance
        for idx in valid_indices_batch:
            if distances_critical[idx - start_idx] > max_distance:
                max_distance = distances_critical[idx - start_idx]
                max_distance_entry = entries[idx]
                max_distance_target = targets[idx]

        # Add the valid indices for the current batch to the overall list
        valid_indices.extend(valid_indices_batch)

    return max_distance_entry, max_distance_target, valid_indices


def get_original_coords(max_distance_entry, max_distance_target):
    entry_final_coord = [
        i for i in entries_coords if torch.all(point_to_numpy_idx(i, list(samples_itk.values())[0][0]) == max_distance_entry)
    ]
    target_final_coord = [
        i for i in targets_coords if torch.all(point_to_numpy_idx(i, list(samples_itk.values())[0][0]) == max_distance_target)
    ]
    assert len(entry_final_coord) == 1, f"Entry final should be a single point: {entry_final_coord}"
    assert len(target_final_coord) == 1, f"Target final should be a single point: {target_final_coord}"
    entry_final_coord, target_final_coord = entry_final_coord[0], target_final_coord[0]
    print(f"Entry final coord: {entry_final_coord}")
    print(f"Target final coord: {target_final_coord}")

    json_content = {
        "original_coordinates": (entry_final_coord.tolist(), target_final_coord.tolist()),
        "in_array_space": (max_distance_entry.tolist(), max_distance_target.tolist()),
    }

    with open("entry_target.json", "w") as f:
        json.dump(json_content, f)


if __name__ == "__main__":
    """
    # depending on your cuda memory if it doesn't work, try reduce the batch size
    # in my case a batch size of 1500 works for the test set and 100 for real set
    # on my machine test set took roughly 1 minute, and real took 20 minutes
    # my gpu has 12gb of memory
    """
    max_distance_entry, max_distance_target, indices = find_max_distance_and_indices(
        entires,
        targets,
        images_meshes,
        batch_size=100,
    )

    print(f"Amount of valid indices after filtering: {len(indices)}")
    assert max_distance_entry is not None, "No valid indices found"
    assert max_distance_target is not None, "No valid indices found"
    print(f"Selected entry: {max_distance_entry}")
    print(f"Selected target: {max_distance_target}")
    get_original_coords(max_distance_entry, max_distance_target)
