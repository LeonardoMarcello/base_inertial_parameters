"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a recorded exciting trajectory
filters the signals, computes the dynamic regressor for a reduced
parameter set, and estimates parameters by least-squares.

"""
from utils.Identifier import *
import utils.import_thunder as thunder


# Set up robot model ------
config_path = '/path/to/identification/config/identif_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
robot = thunder.thunder_['robot']()
thunder.load_params(robot, config['robot']['path'])

# Setup and solve identification Problem ----
identifier = Identifier(robot, config_path=config_path)
identifier.init()                      # 1_ load and process trajectory
identifier.solve_base_parameter()      # 2_ compute parameters in the base
identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
identifier.print_table()               # 4_ print identified dynamics parameters
identifier.save_plot()                 # 5_ save plot
identifier.export()                    # 6_ export in thunder config yaml file
