from src.modules.fcsv import FcsvParser
from pathlib import Path
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

entires_fcsv = FcsvParser(Path("data", "entries.fcsv"))
targets_fcsv = FcsvParser(Path("data", "targets.fcsv"))

entries_coords = torch.tensor(entires_fcsv.content_dataframe[["x", "y", "z"]].values, dtype=torch.float64).to(device)

targets_coords = torch.tensor(targets_fcsv.content_dataframe[["x", "y", "z"]].values, dtype=torch.float64).to(device)
