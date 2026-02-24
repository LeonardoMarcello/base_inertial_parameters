import utils.import_thunder as thunder
from utils.Identifier import *


import utils.import_thunder as thunder

np.set_printoptions(precision=4, suppress=True, linewidth=200)

# Set up Franka Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/franka_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)

franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])

# Setup Identifier Object and Solve Identification Problem ----
Identifier = Identifier(franka, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.solve_base_parameter()      # 2_ compute parameters in the base
Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.export()                    # 4_ store in thunder config yaml file