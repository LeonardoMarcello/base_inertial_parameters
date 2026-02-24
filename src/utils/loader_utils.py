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


import matplotlib.pyplot as plt
from scipy.signal import butter, decimate, filtfilt

from utils.casadi_utils import *


class TrajectoryManager:
    def __init__(self, config_file, filter_order = 4, cutoff_freq = 30, decimate_factor = 10):
        self.df = None
        self.t = None
        self.tau = None
        self.q = None
        self.qd = None
        self.qdd = None

        self.raw_t = None
        self.raw_tau = None
        self.raw_q = None
        self.raw_qd = None
        self.raw_qdd = None

        self.config = {}
        self._load_config(config_file)

        self.filter_order = self.config['processing'].get('butterworth_order', filter_order)
        self.cutoff_freq_rad = self.config['processing'].get('cut_off_frequency', cutoff_freq)
        self.decimate_factor = self.config['processing'].get('decimate_factor', decimate_factor)
        # self.filter_order = filter_order
        # self.cutoff_freq_rad = cutoff_freq
        # self.decimate_factor = decimate_factor
        return


    def process(self, interval_min=5, interval_max=10000, apply_decimation=True):
        # process datafame
        INTERVAL = [interval_min, interval_max]

        # Align timestamps and select the time window of interest
        t0 = self.df['t'].to_numpy()[0]
        self.df = self.df[(self.df['t'] - t0 >= INTERVAL[0]) & (self.df['t'] - t0 <= INTERVAL[1])]

        self.raw_t, idx = np.unique(self.df['t'].to_numpy() - t0, return_index=True)
        self.raw_q = self.df[[col for col in self.df.columns if col.startswith('pos_')]].to_numpy()[idx,:]
        self.raw_qd = self.df[[col for col in self.df.columns if col.startswith('vel_')]].to_numpy()[idx,:]
        self.raw_tau = self.df[[col for col in self.df.columns if col.startswith('eff_')]].to_numpy()[idx,:]

        # Sampling info and backups of original (pre-filter) signals
        t_log = self.raw_t.copy()
        if self.config['processing'].get('flip_torques', False):
            self.raw_tau = -self.raw_tau
        tau_log = self.raw_tau.copy()
        q_log = self.raw_q.copy()
        qd_log = self.raw_qd.copy()

        dt = np.mean(np.diff(self.raw_t))    # sampling period [s]
        F_s = 1.0 / dt                  # sampling frequency [Hz]

        # Design Butterworth filter
        # ==========================================================================================================================
        # Filter design and numerical differentiation
        order = self.filter_order
        # cutoff_freq in Hz (note original comment: should be ~1.6-6.4 Hz)
        cutoff_freq = self.cutoff_freq_rad/(2*np.pi)
        b, a = butter(order, cutoff_freq / (0.5 * F_s), btype='low')

        # 1) Zero-phase low-pass filter positions to reduce noise before differentiation
        q_filt = filtfilt(b, a, q_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        q_log = q_filt

        # 2) Estimate velocity by centered finite differences on filtered positions if not available
        if qd_log.shape != q_log.shape:
            qd_log = np.zeros_like(q_log)
            qd_log[1:-1,:] = (q_log[2:,:] - q_log[:-2,:])/(t_log[2:] - t_log[:-2])[:,None]
            #qd_log[1:-1,:] = (q_log[2:,:] - q_log[:-2,:])/(2*dt)

        qd_filt = filtfilt(b, a, qd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        qd_log = qd_filt

        # 3) Estimate acceleration by centered differences on velocities
        qdd_log = np.zeros_like(qd_log)
        qdd_log[1:-1,:] = (qd_log[2:,:] - qd_log[:-2,:])/(t_log[2:] - t_log[:-2])[:,None]
        #qdd_log[1:-1,:] = (qd_log[2:,:] - qd_log[:-2,:])/(2*dt)

        qdd_filt = filtfilt(b, a, qdd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        qdd_log = qdd_filt

        # 4) Filter commanded torques/efforts with same filter (zero-phase)
        #q_filt = filtfilt(b, a, q_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        #q_log = q_filt
        #qd_filt = filtfilt(b, a, qd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        #qd_log = qd_filt
        #qdd_filt = filtfilt(b, a, qdd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        #qdd_log = qdd_filt
        tau_filt = filtfilt(b, a, tau_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        tau_log = tau_filt

        self.t = t_log
        self.q = q_log
        self.qd = qd_log
        self.qdd = qdd_log
        self.tau = tau_log

        # decimate
        if apply_decimation:
            self.q   = np.zeros((math.ceil(  q_log.shape[0]/self.decimate_factor), q_log.shape[1]))
            self.qd  = np.zeros((math.ceil( qd_log.shape[0]/self.decimate_factor), qd_log.shape[1]))
            self.qdd = np.zeros((math.ceil(qdd_log.shape[0]/self.decimate_factor), qdd_log.shape[1]))
            self.tau = np.zeros((math.ceil(tau_log.shape[0]/self.decimate_factor), tau_log.shape[1]))
            for i in range(q_log.shape[1]):
                self.q[:,i]   = decimate(q_log[:,i], q=self.decimate_factor, zero_phase=True)
                self.qd[:,i]  = decimate(qd_log[:,i], q=self.decimate_factor, zero_phase=True)
                self.qdd[:,i] = decimate(qdd_log[:,i], q=self.decimate_factor, zero_phase=True)
                self.tau[:,i] = decimate(tau_log[:,i], q=self.decimate_factor, zero_phase=True)
            #self.t = decimate(t_log, q = self.decimate_factor, zero_phase=True,axis=0)
            self.t = np.linspace(t_log[0], t_log[-1], num=self.q.shape[0], endpoint=False)

        # Remove border data
        nbord = 5 * self.filter_order
        self.t = self.t[nbord:-nbord]
        self.q = self.q[nbord:-nbord,:]
        self.qd = self.qd[nbord:-nbord,:]
        self.qdd = self.qdd[nbord:-nbord,:]
        self.tau = self.tau[nbord:-nbord,:]

    def plot_traject(self, robot, block = True):
        # --- 1. Plot joint trajectories ---
        n = robot.numJoints
        cols = int(np.sqrt(n)) 
        rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
        JOINT_NAMES = self.config['trajectory']['joints']
        fig_q = plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            plt.plot(self.raw_t, self.raw_q[:,i], 'b', label='q_raw')
            plt.plot(self.t, self.q[:,i], 'r', linestyle='--', label='q')
            plt.title(f'Position {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)
        fig_qd = plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            if (self.raw_qd is not None and 
                self.raw_qd.shape[0] == self.raw_t.shape[0] and
                self.raw_qd.shape[1] == self.raw_q.shape[1]):
                plt.plot(self.raw_t, self.raw_qd[:,i], 'b', label='dq_raw')
            plt.plot(self.t, self.qd[:,i], 'r', label='dq')
            plt.title(f'Velocity {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)
        fig_qdd = plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            plt.plot(self.t, self.qdd[:,i], 'r', label='ddq')
            plt.title(f'Acceleration {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)


        # --- 1. Plot tau ---
        fig_tau = plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            #plt.plot(t_log, tau_ori[:,i], label = 'tau')
            plt.plot(self.raw_t, self.raw_tau[:,i],'b', label = 'tau_raw')
            plt.plot(self.t, self.tau[:,i], 'r', linestyle='dotted', label = 'tau_filtered')
            plt.xlabel('Time [s]')
            plt.ylabel('Torque [Nm]')
            plt.title(f'Torque {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=block)

        return fig_q, fig_qd, fig_qdd, fig_tau



    def plot_traject_recostructed(self, robot, block = True):
        delta_tau = []
        pi_red = robot.get_par_REG_red()
        pi_dl = robot.get_par_Dl()
        for i in range(self.t.shape[0]):
            q = self.q[i,:]
            dq = self.qd[i,:]
            ddq = self.qdd[i,:]

            robot.set_q(q)
            robot.set_dq(dq)
            robot.set_ddq(ddq)
            robot.set_dqr(dq)
            robot.set_ddqr(ddq)

            Y_red = robot.get_Yr_red()
            Y_dl = robot.get_reg_dl()

            tau_est = (Y_red@pi_red + Y_dl@pi_dl).flatten()
            delta_tau.append(self.tau[i,:] - tau_est)

        delta_tau = np.array(delta_tau)


        rmse = np.sqrt(np.mean(np.sum(delta_tau**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
        print(f"RMSE: {rmse} [Nm]" )

        # --- 1. Plot joint trajectories ---
        n = robot.numJoints
        cols = int(np.sqrt(n)) 
        rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
        JOINT_NAMES = self.config['trajectory']['joints']
        plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            plt.plot(self.t, self.q[:,i], 'r', label='q')
            plt.title(f'Position {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)
        plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            plt.plot(self.t, self.qd[:,i], 'r', label='dq')
            plt.title(f'Velocity {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)
        plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            plt.plot(self.t, self.qdd[:,i], 'r', label='ddq')
            plt.title(f'Acceleration {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=False)


        # --- 1. Plot tau ---
        plt.figure(figsize=(12,8))
        for i in range(n):
            plt.subplot(rows, cols,i+1)
            #plt.plot(t_log, tau_ori[:,i], label = 'tau')
            plt.plot(self.t, self.tau[:,i], 'r', label = 'tau_filtered')
            plt.plot(self.t, self.tau[:,i] - delta_tau[:,i], linestyle='dotted', label = 'hat_tau')
            plt.xlabel('Time [s]')
            plt.ylabel('Torque [Nm]')
            plt.title(f'Torque {JOINT_NAMES[i]}')
            plt.grid(True)
            if i==0:
                plt.legend()
        plt.tight_layout()
        plt.show(block=block)

    def export2csv(self, folder_dir):
        """
        Splits the main DataFrame and saves columns into specific identification files.
        :param folder_dir: The directory where the files will be saved.
        """
        # Create the folder if it doesn't exist
        if not os.path.exists(folder_dir):
            os.makedirs(folder_dir)

        # Mapping of column prefixes in the DF to desired filenames
        # Format: { 'column_prefix': 'filename_prefix' }
        mapping = {
            't': 't_identification',
            'pos': 'q_identification',
            'eff': 'tau_identification',
            'vel': 'dq_identification'
        }

        for prefix, filename in mapping.items():
            # Identify columns that start with the prefix (e.g., pos_0, pos_1)
            # We use a filter to catch 't' exactly or 'pos_' specifically
            cols = [c for c in self.df.columns if c == prefix or c.startswith(f"{prefix}_")]

            if cols:
                file_path = os.path.join(folder_dir, f"{filename}.csv")
                self.df[cols].to_csv(file_path, index=False, header=True)
                print(f"Saved {len(cols)} columns to {file_path}")
            else:
                print(f"Note: No columns found for {prefix}, skipping...")


    @staticmethod
    def read_csv(csv_folder):
        # Map prefixes to their desired column naming convention
        prefix_map = {
            'q_': 'pos',
            'eff_': 'eff',
            'dq_': 'vel'
        }
        df = None
        # 1. Handle the time file
        t_path = os.path.join(csv_folder, 't_identification.csv')
        if os.path.exists(t_path):
            df = pd.read_csv(t_path, names=['t'])

        # 2. Iterate through other files and merge
        for filename in os.listdir(csv_folder):
            for prefix, col_label in prefix_map.items():
                if filename.startswith(prefix) and filename.endswith('.csv'):
                    file_path = os.path.join(csv_folder, filename)

                    # Read the data (assuming no header in CSVs)
                    temp_df = pd.read_csv(file_path, header=None)

                    # Rename columns to label_0, label_1, etc.
                    temp_df.columns = [f"{col_label}_{i}" for i in range(temp_df.shape[1])]

                    # Combine with the main dataframe
                    if df is not None:
                        df = pd.concat([df, temp_df], axis=1)
                    else:
                        df = temp_df

        return df


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




    def _load_config(self, filename):
        with open(filename, 'r') as f:
            self.config = yaml.load(f, Loader=SafeLoader)