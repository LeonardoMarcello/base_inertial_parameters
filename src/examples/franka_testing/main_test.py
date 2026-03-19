"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a rosbag, filters the signals,
computes the dynamic regressor for a reduced parameter set, and
estimates parameters by least-squares. It also computes simple
friction terms (viscous + smooth static) and plots results.

The code relies on helper functions and classes from `utils`.
"""
from utils.Identifier import *
import utils.import_thunder as thunder

np.set_printoptions(precision=4, suppress=True, linewidth=200)
opts = {
   "ipopt": {
         "tol": 1e-15,
         "constr_viol_tol": 1e-15,
         "acceptable_tol": 1e-15,
         "dual_inf_tol": 1e-15,
         "compl_inf_tol": 1e-15,
         "max_iter": 50000,
   }
}

# Set up Thumb Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/identif_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])
# Setup Identifier Object and Solve Identification Problem ----
Identifier = Identifier(franka, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.solve_base_parameter()      # 2_ compute parameters in the bas
Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.print_table()               # 4_ print identified dynamics parameters
Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results")     # 5_ save plot
Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results")        # 6_ export in thunder config yaml file


# Identifier.trajectory.export2csv("/home/leo/Desktop/Base Inertial Parameter/src/examples/data")