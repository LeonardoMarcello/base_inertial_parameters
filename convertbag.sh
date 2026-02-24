#!/bin/bash

# Usage:
# ./convertbag.sh '/home/leo/Desktop/franka_ros1_ws/src/panda_controllers/rosbag/backstepping_35s_adpDir_false_RED_true_L_1_02_R_01_Kd_10_5_2_kinDH.bag' './rosbag/backstepping_35s_adpDir_false_RED_true_L_1_02_R_01_Kd_10_5_2_kinDH'

# --- VARIABLES ---
# Edit these paths or pass them as arguments
SRC_BAG="${1:-/path/to/source/robot_data.bag}"
DST_DIR="${2:-/path/to/destination/ros2_bag_folder}"

# --- VALIDATION ---
if [ ! -f "$SRC_BAG" ]; then
    echo "Error: Source bag file '$SRC_BAG' does not exist."
    exit 1
fi

if [ -d "$DST_DIR" ]; then
    echo "Warning: Destination directory '$DST_DIR' already exists."
    echo "Conversion might fail if the folder is not empty."
fi

# --- CONVERSION ---
echo "------------------------------------------------"
echo "Starting conversion:"
echo "Source: $SRC_BAG"
echo "Destination: $DST_DIR"
echo "------------------------------------------------"

# Run the conversion
# --dst creates a folder containing metadata.yaml and the .mcap/.db3 file
rosbags-convert --src "$SRC_BAG" --dst "$DST_DIR"