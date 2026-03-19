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
robot = thunder.thunder_robot()
thunder.load_params(robot, config['robot']['path'])

# Setup and solve identification Problem ----
Identifier = Identifier(robot, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.solve_base_parameter()      # 2_ compute parameters in the base
Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.print_table()               # 4_ print identified dynamics parameters
Identifier.save_plot()                 # 5_ save plot
Identifier.export()                    # 6_ export in thunder config yaml file


# Identifier.trajectory.export2csv("/home/leo/Desktop/Base Inertial Parameter/src/examples/data")