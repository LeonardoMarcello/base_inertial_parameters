import utils.import_thunder as thunder
from utils.evaluation_utils import *
from utils.Identifier import *

np.set_printoptions(precision=4, suppress=True, linewidth=200)
opts = {
   "ipopt": {
         "tol": 1e-8,
         "constr_viol_tol": 1e-8,
         "acceptable_tol": 1e-8,
         "dual_inf_tol": 1e-8,
         "compl_inf_tol": 1e-8,
         "max_iter": 50000,
   }
}
# Set up Finger Model ------
config_path = './src/examples/allegro_hand/config/ahand_finger_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)

finger = thunder.thunder_['ahand_finger']()
thunder.load_params(finger, config['robot']['path'])
# Setup Identifier Object and Solve Identification Problem ----
identifier = Identifier(finger, config_path=config_path)

identifier.init()                      # 1_ load and process trajectory
identifier.solve_base_parameter()      # 2_ compute parameters in the base
print_base_inertial_parameters(finger) # print base inertial parameters before identification
print(finger.get_par_REG_red())
fig_base = plot_base_identification(robot=finger, traject=identifier.trajectory, metrics=identifier.metrics)
fig_base.savefig(os.path.join("./src/examples/allegro_hand/finger/results/base", 'identification_result.png'), bbox_inches='tight', dpi=300)
plt.show(block=True)

identifier.solve_full_dynamics(opts=opts)       # 3_ compute all dynamics parameters
check_feasibility(finger)

identifier.print_table()               # 4_ print identified dynamics parameters
identifier.save_plot(path = "./src/examples/allegro_hand/finger/results")     # 5_ save plot
identifier.export(path = "./src/examples/allegro_hand/finger/results")        # 6_ export in thunder config yaml file

