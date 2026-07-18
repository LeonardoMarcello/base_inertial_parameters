import utils.import_thunder as thunder
from utils.evaluation_utils import *
from utils.Identifier import *


np.set_printoptions(precision=4, suppress=True, linewidth=200)
opts = {
   "ipopt": {
         "tol": 1e-8,
         "constr_viol_tol": 1e-15,
         "acceptable_tol": 1e-15,
         "dual_inf_tol": 1e-15,
         "compl_inf_tol": 1e-15,
         "max_iter": 50000,
   }
}

# Set up Thumb Model ------
config_path = './src/examples/allegro_hand/config/ahand_thumb_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
thumb = thunder.thunder_ahand_thumb()
thunder.load_params(thumb, config['robot']['path'])

# Setup Identifier Object and Solve Identification Problem ----
identifier = Identifier(thumb, config_path=config_path)
identifier.init()                      # 1_ load and process trajectory
identifier.solve_base_parameter()      # 2_ compute parameters in the base
fig_base = plot_base_identification(robot=thumb, traject=identifier.trajectory, metrics=identifier.metrics)
#fig_base.savefig(os.path.join("/home/leo/Desktop/Base Inertial Parameter/src/examples/allegro_hand/thumb/results/base", 'identification_result.png'), bbox_inches='tight', dpi=300)
#plt.show(block=True)

identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
check_feasibility(thumb)

identifier.print_table()               # 4_ print identified dynamics parameters
identifier.save_plot(path = "./src/examples/allegro_hand/thumb/results")     # 5_ save plot
identifier.export(path = "./src/examples/allegro_hand/thumb/results")        # 6_ export in thunder config yaml file



