import json
from pathlib import Path
import numpy as np


def convert_slicer_to_ros(point):
    scale_factor = 0.08
    point = point * scale_factor
    position_correction = np.array([-29.5, -10.5, 0])
    point = point + position_correction
    return np.array([-point[0], point[1], point[2]])


with open(Path("assets", "entry_target_real.json"), "r") as loader:
    data = json.load(loader)

entry, target = data["original_coordinates"][0], data["original_coordinates"][1]
entry, target = np.array(entry), np.array(target)

entry = convert_slicer_to_ros(entry)
target = convert_slicer_to_ros(target)

print(f"entry: {entry}")
print(f"target: {target}")


#  [12.81232, 0.21336 , 9.23592]

# ->
# x: 13.054211475649826
# y: 0.6121101280070368
# z: 9.881893780147616
# [13.05421148, 0.61211013, 9.88189378]
