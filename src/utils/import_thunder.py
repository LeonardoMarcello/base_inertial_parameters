import glob
import os
import sys

# IMPORT THUNDER FILES

# 1) Add installation path
rel_paths = glob.glob("./src/thunder/*/build")

for rel_path in rel_paths:
    if os.path.exists(rel_path):
        sys.path.append(os.path.abspath(rel_path))
    else:
        print(f"Warning: path '{rel_path}' does not exist.")

# 2) Import modules
try:
    from thunder_ahand_thumb_py import thunder_ahand_thumb
    from thunder_ahand_finger_py import thunder_ahand_finger
    from thunder_franka_py import thunder_franka
except ImportError as e:
    print(f"Error: Could not import a thunder module. {e}")

