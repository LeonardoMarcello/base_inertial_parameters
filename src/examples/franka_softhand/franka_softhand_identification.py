"""
This examples contains the identification of the dynamic parameters
of the Franka Emika Panda manipulator with a SoftHand rigidly mounted on
its end-effector. The identification is used considering the solely franka
model with high std. dev. setted for parameters of the last link. 
All data are collected in Gazebo simulative envionment.
"""
from utils.Identifier import *
from utils.evaluation_utils import *
import utils.import_thunder as thunder

np.set_printoptions(precision=4, suppress=True, linewidth=200)
opts = {
   "ipopt": {
         "tol": 1e-15,
         "constr_viol_tol": 1e-15,
         "acceptable_tol": 1e-15,
         "dual_inf_tol": 1e-15,
         "compl_inf_tol": 1e-15,
         "max_iter": 500,
   }
}

# Set up Franka Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_softhand/franka_softhand_identification_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])

franka_ground_truth = thunder.thunder_franka()
thunder.load_params(franka_ground_truth, "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_SH_par.yaml")

# Setup Identifier Object and Solve Identification Problem ----
Identifier = Identifier(franka, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.solve_base_parameter()      # 2_ compute parameters in the base
Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.print_table()               # 4_ print identified dynamics parameters
Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_softhand/results")     # 5_ save plot
Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_softhand/results")        # 6_ export in thunder config yaml file

# Comparison with Soft Hand Ground-Truth
fig_comparisoin = plot_link_solution(franka, franka_ground_truth, n = 6, block = True)
fig_comparisoin.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_softhand/results", 'soft_hand_comparison_results.png'), bbox_inches='tight', dpi=300)