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
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman"],
    "font.size": 10,
    "axes.labelsize": 11,
    "legend.fontsize": 10
})
from scipy.signal import butter, decimate, filtfilt

from utils.casadi_utils import *
from typing import Any, Optional

class TrajectoryManager:
    """
    Class for the ease of trajectory management, filtering, and differentiation.
    """
    # Explicitly declaration of class attribute types for IDEs
    df: pd.DataFrame | None
    
    # Post-Processing measurement arrays
    t: np.ndarray | None
    tau: np.ndarray | None
    q: np.ndarray | None
    qd: np.ndarray | None
    qdd: np.ndarray | None

    # Pre-Processing measurement arrays
    raw_t: np.ndarray | None
    raw_tau: np.ndarray | None
    raw_q: np.ndarray | None
    raw_qd: np.ndarray | None
    raw_qdd: np.ndarray | None

    config: dict[str, Any]
    filter_order: int
    cutoff_freq_rad: float
    decimate_factor: int
    dt: float
    F_s: float


    def __init__(self, config_file:str, filter_order:int = 4, cutoff_freq:float = 30, decimate_factor:int = 10) -> None:
        """
        Initialize the trajectory manager class.

        Args:
            config_file (str): File system path targeting the pipeline configuration description file.
            filter_order (int, optional): Default order for the Butterworth filter if not specified in the configuration. Defaults to 4.
            cutoff_freq (float, optional): Default cut-off frequency [rad/s] if not specified in the configuration. Defaults to 30.0.
            decimate_factor (int, optional): Default downsampling decimation factor if not specified in the configuration. Defaults to 10.
        """
        self.df = None      # Dataframe with raw measurement

        self.t = None       # Post-Processing measurement arary
        self.tau = None
        self.q = None
        self.qd = None
        self.qdd = None

        self.raw_t = None       # Pre-Processing measurement arary
        self.raw_tau = None
        self.raw_q = None
        self.raw_qd = None
        self.raw_qdd = None


        # Trajectory configuration dictionary
        self.config = {}
        self._load_config(config_file)

        # Trajectory processing parameters
        self.filter_order = self.config['processing'].get('butterworth_order', filter_order)
        self.cutoff_freq_rad = self.config['processing'].get('cut_off_frequency', cutoff_freq)
        self.decimate_factor = self.config['processing'].get('decimate_factor', decimate_factor)
        self.dt = 0        # sampling period [s]
        self.F_s = 0       # sampling frequency [Hz]

        return


    def process(self)-> None:
        """
        Process the raw dataframe using standard sequential steps: window trimming, 
        zero-phase Butterworth filtering, centered finite differences, and downsampling.
        """
        # process datafame
        INTERVAL = self.config['trajectory'].get('interval',(5,10000))
        apply_decimation = self.config['processing'].get('apply_decimation',True)

        # 1) Get raw data ----------------
        # Align timestamps and select the time window of interest
        t0 = self.df['t'].to_numpy()[0]
        self.df = self.df[(self.df['t'] - t0 >= INTERVAL[0]) & (self.df['t'] - t0 <= INTERVAL[1])]

        self.raw_t, idx = np.unique(self.df['t'].to_numpy() - t0, return_index=True)
        self.raw_q = self.df[[col for col in self.df.columns if col.startswith('pos_')]].to_numpy()[idx,:]

        if self.config['processing'].get('compute_velocities', False):
            self.raw_qd = np.array([])
        else:
            self.raw_qd = self.df[[col for col in self.df.columns if col.startswith('vel_')]].to_numpy()[idx,:]
        self.raw_tau = self.df[[col for col in self.df.columns if col.startswith('eff_')]].to_numpy()[idx,:]

        # Sampling info and backups of original (pre-filter) signals
        t_log = self.raw_t.copy()
        if self.config['processing'].get('flip_torques', False):
            self.raw_tau = -self.raw_tau
        tau_log = self.raw_tau.copy()
        q_log = self.raw_q.copy()
        qd_log = self.raw_qd.copy()

        dt = np.mean(np.diff(self.raw_t))   # sampling period [s]
        F_s = 1.0 / dt                      # sampling frequency [Hz]

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

        qd_filt = filtfilt(b, a, qd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        qd_log = qd_filt

        # 3) Estimate acceleration by centered differences on velocities
        qdd_log = np.zeros_like(qd_log)
        qdd_log[1:-1,:] = (qd_log[2:,:] - qd_log[:-2,:])/(t_log[2:] - t_log[:-2])[:,None]

        qdd_filt = filtfilt(b, a, qdd_log, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
        qdd_log = qdd_filt

        # 4) Filter commanded torques/efforts with same filter (zero-phase)
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

    def process_pipeline(self) -> None:
        """
        Process raw dataset signals dynamically following a user-configured list array order pipeline.
        """
        processed_data = {}
        self._load_df()
        
        processed_data['t'] = self.raw_t.copy()
        processed_data['tau'] = self.raw_tau.copy()
        processed_data['q'] = self.raw_q.copy()
        
        pipeline = self.config['processing'].get('pipeline',['filtering',
                                                             'differentiating',
                                                             'filtering',
                                                             'differentiating',
                                                             'filtering',
                                                             'decimating',])
        # Processing pipeine
        for elem in pipeline:
            if elem=='filtering':
                processed_data = self._filtering(processed_data)
            if elem=='decimating':
                processed_data = self._decimating(processed_data)
            if elem=='differentiating':
                processed_data = self._differentiating(processed_data)

        # Store processed data
        self.t = processed_data['t']
        self.q = processed_data['q']
        self.qd = processed_data['dq']
        self.qdd = processed_data['ddq']
        self.tau = processed_data['tau']

    def _load_df(self)-> None:
        """
        Extract active signal sections from the loaded DataFrame internal storage.
        """
        # process datafame
        INTERVAL = self.config['trajectory'].get('interval', [0,1000])

        # 1) Get raw data ----------------
        # Align timestamps and select the time window of interest
        t0 = self.df['t'].to_numpy()[0]
        self.df = self.df[(self.df['t'] - t0 >= INTERVAL[0]) & (self.df['t'] - t0 <= INTERVAL[1])]
        # times
        self.raw_t, idx = np.unique(self.df['t'].to_numpy() - t0, return_index=True)
        # position
        self.raw_q = self.df[[col for col in self.df.columns if col.startswith('pos_')]].to_numpy()[idx,:]
        # velocities
        if self.config['processing'].get('compute_velocities', False):
            self.raw_qd = np.array([])
        else:
            self.raw_qd = self.df[[col for col in self.df.columns if col.startswith('vel_')]].to_numpy()[idx,:]
        # effort
        self.raw_tau = self.df[[col for col in self.df.columns if col.startswith('eff_')]].to_numpy()[idx,:]
        if self.config['processing'].get('flip_torques', False):
            self.raw_tau = -self.raw_tau

        self.dt = np.mean(np.diff(self.raw_t))        # sampling period [s]
        self.F_s = 1.0 / self.dt                      # sampling frequency [Hz]


    def _filtering(self, data: dict) -> dict:
        """
        Filter the parsed tracking signal data arrays using a low-pass zero-phase Butterworth implementation.

        Args:
            data (dict): Dictionary mapping signal names ('t', 'q', 'tau', etc.) to their respective NumPy arrays.

        Returns:
            dict: The modified dictionary containing the filtered and border-trimmed data channels.
        """
        filtered_data = data.copy()

        # Filter design and numerical differentiation
        order = self.filter_order
        nbord = 5 * self.filter_order # used for removing border filtering effect
        # cutoff_freq in Hz (note original comment: should be ~1.6-6.4 Hz)
        cutoff_freq = self.cutoff_freq_rad/(2*np.pi)
        b, a = butter(order, cutoff_freq / (0.5 * self.F_s), btype='low')

        for key, array in data.items():
            if key != 't':
                # filtered all elements parsed
                filtered_data[key] = filtfilt(b, a, array, axis=0, padtype="odd", padlen=3 * (max(len(b), len(a)) - 1))
            else:
                filtered_data[key] = array
            # remove border data
            filtered_data[key] = filtered_data[key][nbord:-nbord]

        return filtered_data

    def _decimating(self, data: dict) -> dict:
        """
        Downsample the active configuration channels according to the system decimation factor.

        Args:
            data (dict): Dictionary mapping signal names to their uniform length tracking arrays.

        Returns:
            dict: The downsampled sequence arrays dictionary with updated frequencies.
        """
        decimated_data = data.copy()
        # 1) Check parsed data
        # - Check that times are available
        if 't' not in data.keys():
            raise KeyError("Decimating Error: Times array must be parsed in data using 't' keys")
        # - Check data dimensions
        array_len =  data['t'].shape[0]
        for key, array in data.items():
            if array_len !=  array.shape[0]:
                raise ValueError(f"Decimating Error: '{key}' has a length not compatible with times data")

        # 2) Apply decimation
        decimated_array_len = math.ceil(array_len/self.decimate_factor)
        for key, array in data.items():
            if key!='t':
                decimated_data[key] = np.zeros((decimated_array_len, array.shape[1]))
                for i in range(array.shape[1]):
                    # decimate i-th joints
                    decimated_data[key][:,i] = decimate(array[:,i], q=self.decimate_factor, zero_phase=True)
            else:
                decimated_data['t'] = np.linspace(data['t'][0], data['t'][-1], num = decimated_array_len, endpoint=False)

        # update sampling frequency
        self.dt = self.dt*self.decimate_factor
        self.F_s = self.F_s/self.decimate_factor

        return decimated_data

    def _differentiating(self, data: dict) -> dict:
        """
        Calculate numeric derivations using a central finite difference algorithm schema.

        Args:
            data (dict): Dictionary containing the base tracking paths.

        Returns:
            dict: The differentiated output dictionary containing the newly created key channels.
        """
        differentiated_data = data.copy()
        # 1) Check parsed data
        # - Check that times are available
        if 't' not in data.keys():
            raise KeyError("Differentiating Error: Times array must be parsed in data using 't' keys")
        # - Check data dimensions
        array_len =  data['t'].shape[0]
        time_array =  data['t']
        for key, array in data.items():
            if array_len !=  array.shape[0]:
                raise ValueError(f"Differentiating Error: '{key}' has a length not compatible with times data")

        # 2) Compute the central finite differentiation
        for key, array in data.items():
            if key != 't':
                differentiated_data[key] = array[1:-1,:]
                differentiated_data[f"d{key}"] = np.zeros_like(array[1:-1,:])
                differentiated_data[f"d{key}"] = (array[2:,:] - array[:-2,:])/(time_array[2:] - time_array[:-2])[:,None]
            else:
                differentiated_data["t"] = time_array[1:-1]
        return differentiated_data



    def plot_traject(self, robot: Any, block: bool = True) -> tuple[plt.Figure, plt.Figure, plt.Figure, plt.Figure]:
        """
        Render processing comparison loops tracking unfiltered signals against final filtered trajectory records.

        Args:
            robot (Any): Robot model instance built with the thunder dynamic tool.
            block (bool, optional): If True, blocks code processing flow execution until window close commands. Defaults to True.

        Returns:
            tuple[plt.Figure, plt.Figure, plt.Figure, plt.Figure]: Reconstructed figure tracking sheets (pos, vel, accel, effort).
        """
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


    def compute_ne(self, robot: Any) -> np.ndarray:
        """
        Evaluate Newton-Euler inverse dynamics torques point-by-point along the trajectory.

        Args:
            robot (Any): Target robot tracking model structure matrix properties setup.

        Returns:
            np.ndarray: Matrix containing the reconstructed dynamic torques profiles.
        """
        tau_ne = []
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
            tau_ne.append(tau_est)

        tau_ne = np.array(tau_ne)
        return tau_ne

    def plot_traject_recostructed(self, robot: Any, block: bool = True) -> None:
        """
        Render dynamic parameter tracking accuracy visualizations comparing calculated estimations.

        Args:
            robot (Any): Target robot instance model profile.
            block (bool, optional): If True, blocks script thread execution until closing windows. Defaults to True.
        """
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

    def export2csv(self, folder_dir: str) -> None:
        """
        Split the composite workspace metrics sheet out into standalone component CSV records.

        Args:
            folder_dir (str): Destination folder directory mapping path.
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
    def read_csv(csv_folder: str) -> Optional[pd.DataFrame]:
        """
        Parse dynamic logging matrix values located across multiple custom system text sheets.

        Args:
            csv_folder (str): Directory containing the set of input CSV data files.

        Returns:
            Optional[pd.DataFrame]: Combined Pandas dataframe tracking records structure.
        """
        # Map prefixes to their desired column naming convention
        # Format: { 'filename_prefix': 'dataframe_column_prefix' }
        prefix_map = {
            'q_': 'pos',
            'tau_': 'eff',
            'dq_': 'vel'
        }
        df = None

        # 1. Handle the time file
        t_path = os.path.join(csv_folder, 't_identification.csv')
        if os.path.exists(t_path):
            df = pd.read_csv(t_path, names=['t'])

        # 2. Iterate through other files and merge in the dataframe
        for filename in os.listdir(csv_folder):
            for prefix, col_label in prefix_map.items():
                if filename.startswith(prefix) and filename.endswith('.csv'):
                    file_path = os.path.join(csv_folder, filename)

                    # Read the data
                    temp_df = pd.read_csv(file_path, header=0)

                    # Rename columns to proper labelling in the dataframe
                    temp_df.columns = [f"{col_label}_{i}" for i in range(temp_df.shape[1])]

                    # Combine with the main dataframe
                    if df is not None:
                        df = pd.concat([df, temp_df], axis=1)
                    else:
                        df = temp_df
        return df

    def _load_config(self, filename: str) -> None:
        """
        Parse tracking configuration properties while mapping fallback template baselines.

        Args:
            filename (str): The configuration file destination location mapping.
        """
        # 1. Load the user's YAML file (the "or {}" prevents crashes if the file is totally empty)
        with open(filename, 'r') as f:
            self.config = yaml.load(f, Loader=SafeLoader) or {}
        # 2. Define your default fallback template
        default_template = {
            'trajectory': {
                'interval': [0, 1000]   # min and max interval to process [s]
            },
            'processing': {
                'compute_velocities': False,    # wheter compute velocities via finite diff. whan in available raw data
                'flip_torques': False,          # wheter change sign of torque (e.g. when using franka join states data)
                'pipeline': [                   # pipeline for data processing
                    'filtering',
                    'differentiating',
                    'filtering',
                    'differentiating',
                    'filtering',
                    'decimating'
                ],
                'cut_off_frequency': 30,    # butterworth filter cut-off frequency [rad/s]
                'butterworth_order': 5,     # butterworth filter order [rad/s]
                'decimate_factor': 10,      # decimating ratio
            }
        }
        # 3. Safely merge the defaults without overwriting the user's custom choices
        for section in ['trajectory', 'processing']:
            # If the entire section is missing, copy the whole default block over
            if section not in self.config:
                self.config[section] = default_template[section]
            else:
                # If the section exists, check its inner keys to fill in any missing pieces
                for key, default_value in default_template[section].items():
                    self.config[section].setdefault(key, default_value)
