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
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim/config/franka_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
franka_ground_truth = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])
thunder.load_params(franka_ground_truth, config['robot']['path'])
# Add Gaussian noise to real params ----------------------------------
std = 0.2 # normalized on real values
error = franka.get_par_DYN() * np.random.normal(0, std, franka.get_par_DYN().shape)
franka.set_par_DYN(franka.get_par_DYN() + error)
franka.set_par_REG(franka.get_dyn2reg())
franka.set_par_REG_red(franka.get_reg2red())

# Setup and Solve Identification Problem ----------------------------------
Identifier = Identifier(franka, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
fig = plot_identification(Identifier.robot, Identifier.trajectory, Identifier.metrics, title = "Before Optimization", block = False)  # 4_ plot before optimization
fig.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim/results/franka/before", 'torque.png'), bbox_inches='tight', dpi=300)


print("Number of samples ", Identifier.trajectory.raw_tau.shape[0])
Identifier.solve_base_parameter()      # 2_ compute parameters in the base


Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.print_table()               # 4_ print identified dynamics parameters
Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim/results/franka")     # 5_ save plot
Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_sim/results/franka")

# Comparison with Ground-Truth
check_feasibility(franka)
print_base_inertial_parameters(franka)
plot_table(franka, franka_ground_truth, format='latex')