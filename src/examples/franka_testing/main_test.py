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


hsllib_path = "/usr/local/lib/x86_64-linux-gnu/libcoinhsl.so" 
np.set_printoptions(precision=4, suppress=True, linewidth=200)
opts = {
   "ipopt": {
         "tol": 1e-15,
         "constr_viol_tol": 1e-10,
         "acceptable_tol": 1e-10,
         "dual_inf_tol": 1e-10,
         "compl_inf_tol": 1e-10,
         "max_iter": 5000,
   }
}
opts = {
   "ipopt": {
         # The main tolerance. 1e-6 is highly accurate but numerically safe.
         "tol": 1e-6, 
         # How much a constraint is allowed to be violated. 
         # 1e-6 means 0.000001, which is plenty strict for inertia!
         "constr_viol_tol": 1e-6, 
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


# Set up Thumb Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/identif_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])
#Add Gaussian noise to real params ----------------------------------
np.random.seed(42)
std = 0.5 # normalized on real values
error = franka.get_par_DYN() * np.random.normal(0, std, franka.get_par_DYN().shape)
franka.set_par_DYN(franka.get_par_DYN() + error)
franka.set_par_REG(franka.get_dyn2reg())
franka.set_par_REG_red(franka.get_reg2red())


franka_ground_truth = thunder.thunder_franka()
#thunder.load_params(franka_ground_truth, "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_SH_par.yaml")
thunder.load_params(franka_ground_truth, "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_par.yaml")

# check_par_DYN_feasibility(franka.get_par_DYN(),7, verbose=True)
# res = LMI_projection(franka)
# check_par_DYN_feasibility(res,7, verbose=True)
# exit()
print_base_inertial_parameters(franka)
# Setup Identifier Object and Solve Identification Problem ----
Identifier = Identifier(franka, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results/before")     # 5_ save plot

Identifier.solve_base_parameter()      # 2_ compute parameters in the base
print("Trajectory conditioning number:")
print(Identifier.metrics['conditioning number'].T) # <- conditioining number
print("Trajectory sigma max:")
print(Identifier.metrics['sigma max'].T) # <- max std.dev.
print("Covariance matrix of the estimation error (parameters relative standard deviation):")
print(Identifier.metrics['parameters relative standard deviation'].T) # <- covariance matrix
print("Estimated reduced parameters (percentage):")
print(np.abs(franka_ground_truth.get_par_REG_red()-franka.get_par_REG_red())/np.abs(franka_ground_truth.get_par_REG_red()).T) # <- percentage error
exit(0)

fig_base = plot_base_identification(robot=franka, traject=Identifier.trajectory, metrics=Identifier.metrics)
#fig_base.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results/base", 'identification_result.png'), bbox_inches='tight', dpi=300)

Identifier.solve_full_dynamics(opts=opts)       # 3_ compute all dynamics parameters
Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results")     # 5_ save plot

check_feasibility(franka)
plot_table(franka, franka_ground_truth)
plt.show(block=True)
Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results")        # 6_ export in thunder config yaml file



# Comparison with Soft Hand Ground-Truth
fig_comparisoin = plot_link_solution(franka, franka_ground_truth, n = 6, block = True)
#fig_comparisoin.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results", 'soft_hand_comparison_results.png'), bbox_inches='tight', dpi=300)
# Identifier.trajectory.export2csv("/home/leo/Desktop/Base Inertial Parameter/src/examples/data")