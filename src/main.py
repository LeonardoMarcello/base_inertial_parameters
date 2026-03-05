import utils.import_thunder as thunder
from utils.Identifier import *


import utils.import_thunder as thunder

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
# # Set up Franka Model ------
# config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/franka_config.yaml'
# with open(config_path, 'r') as f:
#    config = yaml.load(f, Loader=SafeLoader)
# franka = thunder.thunder_franka()
# thunder.load_params(franka, config['robot']['path'])


# Set up Thumb Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_thumb_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
thumb = thunder.thunder_ahand_thumb()
thunder.load_params(thumb, config['robot']['path'])
# Setup Identifier Object and Solve Identification Problem ----
Identifier = Identifier(thumb, config_path=config_path)
Identifier.init()                      # 1_ load and process trajectory
Identifier.solve_base_parameter()      # 2_ compute parameters in the base
Identifier.solve_full_dynamics()       # 3_ compute all dynamics parameters
Identifier.print_table()               # 4_ print identified dynamics parameters
#Identifier.save_plot(path = "/home/leo/Desktop/Base Inertial Parameter/results/ahand_identification/thumb")     # 5_ save plot
#Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/results/ahand_identification/thumb")        # 6_ export in thunder config yaml file
  


# # Set up Finger Model ------
# config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_finger_config.yaml'
# with open(config_path, 'r') as f:
#    config = yaml.load(f, Loader=SafeLoader)
# finger = thunder.thunder_ahand_finger()
# thunder.load_params(finger, config['robot']['path'])
# # Setup Identifier Object and Solve Identification Problem ----
# Identifier = Identifier(finger, config_path=config_path)
# Identifier.init()                      # 1_ load and process trajectory
# Identifier.solve_base_parameter()      # 2_ compute parameters in the base
# Identifier.solve_full_dynamics(opts=opts)       # 3_ compute all dynamics parameters
# Identifier.print_table()               # 4_ print identified dynamics parameters
# Identifier.save_plot(block=True,path = "/home/leo/Desktop/Base Inertial Parameter/results/ahand_identification/finger")     # 5_ save plot
# Identifier.export(path = "/home/leo/Desktop/Base Inertial Parameter/results/ahand_identification/finger")        # 6_ export in thunder config yaml file
   


