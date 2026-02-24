"""
utilities function for loading trajectory to use in the idientification process.
"""

import os

import numpy as np
import math
import pandas as pd
import yaml
from yaml.loader import SafeLoader

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

from utils.loader_utils import *


class TrajectoryManagerROS2(TrajectoryManager):
    def __init__(self, config_file, filter_order = 4, cutoff_freq = 30, decimate_factor = 10):
        self.super().__init__(config_file, filter_order, cutoff_freq, decimate_factor)
        return

    @staticmethod
    def read_bag(file_path, topic_name, joint_names=None):
        first = True
        # Open the bag reader
        reader = rosbag2_py.SequentialReader()

        # set options and open the bag
        storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                        output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        dataset = []
        header = []
        # scroll the bag
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == topic_name:
            #if topic == '/allegroHand_0/torque_cmd':
                msg = deserialize_message(data, JointState)
                msg_joint_names = msg.name

                # select specific joints
                indices = [msg_joint_names.index(name) for name in joint_names if name in msg_joint_names]
                selected_joint_names = [msg_joint_names[i] for i in indices]
                if first:
                    msg_joint_names = msg.name
                    # get header
                    pos_header    = [f"pos_{name}" for name in selected_joint_names]
                    effort_header = [f"eff_{name}" for name in selected_joint_names]
                    vel_header    = [f"vel_{name}" for name in selected_joint_names]

                    header = ['t'] + pos_header + effort_header
                    if len(msg.velocity) == len(msg.position): 
                        header += vel_header
                        print("[INFO] Loading velocities from the topic field header")
                    else:
                        print("[INFO] Velocities not found. they will be obtained by central differentiation")

                    first = False
                elif msg_joint_names != msg.name:
                    raise ValueError("Joint names do not match")

                # Store data
                try:
                    selected_positions = [msg.position[i] for i in indices]
                    selected_effort = [msg.effort[i] for i in indices]
                    has_vel = any("vel" in col for col in header)
                    if has_vel and len(msg.velocity) == len(msg.position):
                        selected_velocities = [msg.velocity[i] for i in indices]
                        row = [t/1e9] + selected_positions + selected_effort + selected_velocities
                    else:
                        row = [t/1e9] + selected_positions + selected_effort
                    dataset.append(row)
                except Exception as e:
                    print(e)
                    pass

        # store in dataframe
        return pd.DataFrame(dataset, columns=header)