"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a rosbag, filters the signals,
computes the dynamic regressor for a reduced parameter set, and
estimates parameters by least-squares. It also computes simple
friction terms (viscous + smooth static) and plots results.

The code relies on helper functions and classes from `utils`.
"""
from utils.Identifier import *
from utils.evaluation_utils import *
import utils.import_thunder as thunder


opts = {
   "ipopt": {
         # The main tolerance. 1e-6 is highly accurate but numerically safe.
         "tol": 1e-10, 
         # How much a constraint is allowed to be violated. 
         # 1e-6 means 0.000001, which is plenty strict for inertia!
         "constr_viol_tol": 1e-12, 
         # If the solver gets stuck, it will accept a slightly worse 
         # solution (1e-4) instead of crashing.
         "acceptable_tol": 1e-4,
         "acceptable_iter": 15, # Accept after 15 iterations of being "good enough"
         # Don't let dual variables trigger a crash when hitting boundaries
         "dual_inf_tol": 1e-4, 
         "compl_inf_tol": 1e-4,
         "max_iter": 5000, # 50000 is fine, but if it takes >5000, something is wrong.
   }
}

# --------


# Set up Franka Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_real/config/identif_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])


franka_ground_truth = thunder.thunder_franka()
thunder.load_params(franka_ground_truth, "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_par.yaml")


# Setup Identifier Object and Solve Identification Problem ----
identifier = Identifier(franka, config_path=config_path)
identifier.init()                      # 1_ load and process trajectory
identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_real/results/before")     # 5_ save plot


identifier.solve_base_parameter()      # 2_ compute parameters in the base
fig_base = plot_base_identification(robot=franka, traject=identifier.trajectory, metrics=identifier.metrics)
#fig_base.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results/base", 'identification_result.png'), bbox_inches='tight', dpi=300)

identifier.solve_full_dynamics(opts=opts)       # 3_ compute all dynamics parameters
identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_real/results")     # 5_ save plot
check_feasibility(franka)
plot_table(franka, franka_ground_truth)
plt.show(block=True)
identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_real/results")        # 6_ export in thunder config yaml file