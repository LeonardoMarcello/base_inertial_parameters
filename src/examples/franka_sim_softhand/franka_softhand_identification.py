"""
This example includes the estimation of the Franka model
when all the set of dynamic parameter are not exactly known
"""
from utils.Identifier import *
from utils.evaluation_utils import *
import utils.import_thunder as thunder

np.random.seed(41)
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
# Set up Franka Model ----------------------------------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim_softhand/config/softhand_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
franka_ground_truth = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])
thunder.load_params(franka_ground_truth, "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_SH_par.yaml")

# Setup and Solve Identification Problem ----------------------------------
identifier = Identifier(franka, config_path=config_path)
identifier.init()                      # 1_ load and process trajectory
fig = plot_identification(identifier.robot, identifier.trajectory, identifier.metrics, title = "Before Optimization", block = False)  # 4_ plot before optimization
fig.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim_softhand/results/before", 'torque.png'), bbox_inches='tight', dpi=300)


print("Number of samples ", identifier.trajectory.raw_tau.shape[0])
identifier.solve_base_parameter()      # 2_ compute parameters in the base


identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
identifier.print_table()               # 4_ print identified dynamics parameters
identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim_softhand/results/")     # 5_ save plot
identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim_softhand/results/")

# Comparison with Ground-Truth
check_feasibility(franka)
fig_comparisoin = plot_link_solution(franka, franka_ground_truth, n = 6, block = True)
fig_comparisoin.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim_softhand/results/", 'soft_hand_comparison_results.png'), bbox_inches='tight', dpi=300)
plot_table(franka, franka_ground_truth, format='latex')