"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a rosbag, filters the signals,
computes the dynamic regressor for a reduced parameter set, and
estimates parameters by least-squares. It also computes simple
friction terms (viscous + smooth static) and plots results.

The code relies on helper functions and classes from `utils`.
"""
#from utils.BaseParameter import *
import utils.import_thunder as thunder

import numpy as np
import matplotlib.pyplot as plt

from utils.loader_utils import *
from utils.identification_utils import *


# 1. Define and create the results directory
date_str = datetime.datetime.now().strftime("%d_%m_%Y")
results_dir = f"results/data_{date_str}"
if not os.path.exists(results_dir):
        os.makedirs(results_dir)
# ---------------------------------------------------


np.random.seed(41)
np.set_printoptions(precision=4, suppress=True, linewidth=200)

# read config

# Franka ------
SOFTHAND = False
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/franka_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
robot = thunder.thunder_franka()
robot_ig = thunder.thunder_franka()
robot_gt = thunder.thunder_franka()


# Thumb ------
# with open('/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_config.yaml', 'r') as f:
#    config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_config.yaml'
#    config = yaml.load(f, Loader=SafeLoader)
# robot = thunder.thunder_ahand_thumb()
# robot_ig = thunder.thunder_ahand_thumb()

# Finger ------
# with open('/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_finger_config.yaml', 'r') as f:
#     config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/ahand_finger_config.yaml'
#     config = yaml.load(f, Loader=SafeLoader)
# robot = thunder.thunder_ahand_finger()
# robot.set_par_gravity([0,0.001,-9.81]) # MX_0 and MY_0 - MZ_1 - MZ_2 - MZ_3 no obs with vertical gravity
# robot_ig = thunder.thunder_ahand_finger()

# load robot params
config_robot = config['robot']
try:
    # Set initial guess
    load_params(robot, config_robot['path'])
    load_params(robot_ig, config_robot['path'])

    # Set ground truth
    if SOFTHAND:
        gt_params_path = "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_SH_par.yaml"
    else:
        gt_params_path = "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_par.yaml"
    load_params(robot_gt, gt_params_path)

    # Add initial guess with noise
    std = 0.1
    #std = 0
    robot.set_par_DYN(robot.get_par_DYN()  + robot.get_par_DYN() * np.random.normal(0, std, robot.get_par_DYN().shape))
    robot.set_par_REG(robot.get_dyn2reg())
    robot.set_par_REG_red(robot.get_reg2red())
    robot_ig.set_par_DYN(robot.get_par_DYN())
    robot_ig.set_par_REG(robot.get_par_REG())
    robot_ig.set_par_REG_red(robot.get_par_REG_red())
except Exception as e:
    raise e

# Load Traject
config_traject = config['trajectory']
config_processing = config['processing']
config_iden = config['identification']
#test_traject = TestTraject(config_processing['butterworth_order'],config_processing['cut_off_frequency'], config_processing['decimate_factor'])
test_traject = TrajectManager(config_path, config_processing['butterworth_order'],config_processing['cut_off_frequency'], config_processing['decimate_factor'])

if config_traject['type'] == 'ros2':
    test_traject.df = TrajectManager.read_bag(config_traject['bag'], config_traject['topic'], config_traject['joints'])
elif config_traject['type'] == 'csv':
    test_traject.df = TrajectManager.read_csv(config_traject['path'])


config_traject_interval = config_traject.get('interval', (5,100))
apply_decimation = config_processing.get('apply_decimation', False)
test_traject.process(interval_min=config_traject_interval[0], interval_max=config_traject_interval[1], apply_decimation=apply_decimation)  



fig_q, fig_qd, fig_qdd, fig_tau = test_traject.plot_traject(robot)


# Base inertial parameters
print("\nBase Inertial Parameters")
bp = print_base_inertial_parameters(robot)
# =========================================================================================================================== Compute regressor
# ==========================================================================================================================
YY, TTau = get_big_Y_Tau(robot, test_traject)

print("> Computing Dynamics param")
print("")
n = robot.numJoints
M = len(test_traject.t)
p = YY.shape[1]

print("Number of samples (M): ", M)
print("Number of parameters in the base: ", p)
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)


# =========================================================================================================================== Compute Metrics
# ==========================================================================================================================

conditioning_ratio = config_iden.get('conditioning_ratio', None)

# Method 1 - OLS ---------------------------------------------------------------
print("\nComputing reduced pi solution by OLS")
#hat_Pi = np.linalg.pinv(YY) @ TTau

hat_Pi = solve_OLS(robot, test_traject, conditioning_ratio = conditioning_ratio)
if config_iden['method'] == 'OLS|prior':
    hat_Pi = solve_OLS_with_prior(robot, test_traject, conditioning_ratio = conditioning_ratio)

metrics = get_metrics(robot, test_traject, hat_Pi=hat_Pi)

pi_red_CAD = robot_ig.get_par_REG_red()
pi_dl_CAD = robot_ig.get_par_Dl()


print("The conditioning number is", metrics['conditioning number'])
# OLS residulas solution
residual = metrics['residual']

# stdev of residual error ro
sigma_w = metrics['error standard deviation']

# covariance matrix of estimated parameters
C_w = metrics['parameters covariance matrix']

# relative stdev of estimated parameters
sigma_pi = metrics['parameters standard deviation']
sigma_pi_perc = metrics['parameters relative standard deviation']

print("Parameters std. deviation (%):")
print(sigma_pi_perc.flatten())

# Compute Essential Parameters
compute_SVD_essential(robot, test_traject, conditioning_ratio = 30)
hat_Pi_e, idx_e = compute_essential(robot, test_traject, ratio_essential = 30)
# metrics = get_metrics(robot,test_traject,hat_Pi_essential=hat_Pi_e,idx_essental=idx_e)
# sigma_pi_perc = metrics['parameters relative standard deviation']

# hat_Pi = np.zeros_like(hat_Pi)
# hat_Pi[idx_e] = hat_Pi_e
print(f"Essential parameters indexes: {idx_e}, {len(idx_e)} out of {p} (ratio > 30)")
#[print(bp[f"{i}"]) for i in idx_e]
print(f"Essential parameters Conditinoning: {np.linalg.cond(YY[:,idx_e])}")

# Method 2 - weighted LS ---------------------------------------------------------------
print("\nComputing reduced pi solution by WLS")
hat_Pi_OLS = hat_Pi.copy()

if config_iden['method'] == 'WLS':
    hat_Pi = solve_WLS(robot, test_traject, conditioning_ratio = conditioning_ratio)
elif config_iden['method'] == 'WLS|prior':
    hat_Pi = solve_WLS_with_prior(robot, test_traject, conditioning_ratio = conditioning_ratio)

# Assign estimation
hat_pi_REG_red = hat_Pi[:-n*(robot.Dl_order)]
hat_pi_dl = hat_Pi[-n*(robot.Dl_order):]
robot.set_par_REG_red(hat_pi_REG_red)
robot.set_par_Dl(hat_pi_dl)


print(">> Base inertial Parameters Estimation")
print("")
# Compute residual norm and singular values for analysis
Pi_CAD = np.hstack([pi_red_CAD, pi_dl_CAD]).reshape(-1,1)
residual = np.linalg.norm(TTau - YY @ Pi_CAD)
print(f"Residuals (CAD): {residual} [Nm]" )

residual = np.linalg.norm(TTau - YY @ hat_Pi_OLS)
print(f"Residuals (OLS): {residual} [Nm]" )

residual = np.linalg.norm(TTau - YY @ hat_Pi)
print(f"Residuals: {residual} [Nm]" )

print("Parameters estimation")
print(hat_Pi.flatten())
print(f"error std. deviation: {sigma_w}")
print("Parameters std. deviation:")
print(sigma_pi.flatten())
print("Parameters std. deviation (%):")
print(sigma_pi_perc.flatten())

plot_LS_solution(hat_Pi,metrics, pi_gt = Pi_CAD)

# =========================================================================================================================== Plot REDUCED
# ==========================================================================================================================
# Compute residuals (measured torque - predicted torque) for each sample
delta_tau = []
delta_tau_OLS = []
delta_tau_E = []
delta_tau_CAD = []

pi_red = robot.get_par_REG_red()
pi_dl = robot.get_par_Dl()
pi_red_CAD = robot_gt.get_par_REG_red()
pi_dl_CAD = robot_gt.get_par_Dl()


for i in range(test_traject.t.shape[0]):
    q = test_traject.q[i,:]
    dq = test_traject.qd[i,:]
    ddq = test_traject.qdd[i,:]

    robot.set_q(q)
    robot.set_dq(dq)
    robot.set_ddq(ddq)
    robot.set_dqr(dq)
    robot.set_ddqr(ddq)

    Y_red = robot.get_Yr_red()
    Y_dl = robot.get_reg_dl()
    Y_ = np.hstack([Y_red, Y_dl])

    tau_est = (Y_red@pi_red + Y_dl@pi_dl).flatten()
    delta_tau.append(test_traject.tau[i,:] - tau_est)
    delta_tau_OLS.append(test_traject.tau[i,:] - (Y_red@hat_Pi_OLS[:-n*(robot.Dl_order)] + Y_dl@hat_Pi_OLS[-n*(robot.Dl_order):]).flatten())
    delta_tau_E.append(test_traject.tau[i,:] - (Y_[:,idx_e] @ hat_Pi_e).flatten())
    delta_tau_CAD.append(test_traject.tau[i,:] - (Y_red@pi_red_CAD + Y_dl@pi_dl_CAD).flatten())
delta_tau = np.array(delta_tau)
delta_tau_OLS = np.array(delta_tau_OLS)
delta_tau_E = np.array(delta_tau_E)
delta_tau_CAD = np.array(delta_tau_CAD)


rmse = np.sqrt(np.mean(np.sum(delta_tau**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
print(f"RMSE: {rmse} [Nm]" )
rmse = np.sqrt(np.mean(np.sum(delta_tau_OLS**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
print(f"RMSE (OLS): {rmse} [Nm]" )
rmse = np.sqrt(np.mean(np.sum(delta_tau_E**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
print(f"RMSE (E): {rmse} [Nm]" )
rmse = np.sqrt(np.mean(np.sum(delta_tau_CAD**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
print(f"RMSE (CAD): {rmse} [Nm]" )



# # --- 1. Plot joint trajectories ---
# cols = int(np.sqrt(n)) 
# rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
# JOINT_NAMES = config_traject['joints']
# plt.figure(figsize=(12,8))
# for i in range(n):
#     plt.subplot(rows, cols,i+1)
#     plt.plot(test_traject.t, test_traject.q[:,i], 'r', label='q')
#     plt.title(f'Position {JOINT_NAMES[i]}')
#     plt.grid(True)
#     if i==0:
#         plt.legend()
# plt.tight_layout()
# plt.show(block=False)
# plt.figure(figsize=(12,8))
# for i in range(n):
#     plt.subplot(rows, cols,i+1)
#     plt.plot(test_traject.t, test_traject.qd[:,i], 'r', label='dq')
#     plt.title(f'Velocity {JOINT_NAMES[i]}')
#     plt.grid(True)
#     if i==0:
#         plt.legend()
# plt.tight_layout()
# plt.show(block=False)
# plt.figure(figsize=(12,8))
# for i in range(n):
#     plt.subplot(rows, cols,i+1)
#     plt.plot(test_traject.t, test_traject.qdd[:,i], 'r', label='ddq')
#     plt.title(f'Acceleration {JOINT_NAMES[i]}')
#     plt.grid(True)
#     if i==0:
#         plt.legend()
# plt.tight_layout()
# plt.show(block=False)


# --- 1. Plot tau ---
cols = int(np.sqrt(n)) 
rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
JOINT_NAMES = config_traject['joints']
fig_fitting = plt.figure(figsize=(12,8))
for i in range(n):
    plt.subplot(rows, cols,i+1)
    #plt.plot(t_log, tau_ori[:,i], label = 'tau')
    plt.plot(test_traject.t, test_traject.tau[:,i], 'r', label = 'tau_filtered')
    plt.plot(test_traject.t, test_traject.tau[:,i] - delta_tau[:,i],  label = 'hat_tau')
    plt.plot(test_traject.t, test_traject.tau[:,i] - delta_tau_OLS[:,i], linestyle='dashed', label = 'hat_tau_OLS')
    plt.plot(test_traject.t, test_traject.tau[:,i] - delta_tau_E[:,i], linestyle='dashed', label = 'hat_tau_E')
    plt.plot(test_traject.t, test_traject.tau[:,i] - delta_tau_CAD[:,i], linestyle='dashed', label = 'hat_tau_CAD')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Torque {JOINT_NAMES[i]}')
    plt.grid(True)
    if i==0:
        plt.legend()
plt.tight_layout()
plt.show()

# =========================================================================================================================== CasADi Solver
# ==========================================================================================================================
import casadi
from utils.casadi_utils import *
import random
random.seed(41)


#robot.set_par_REG_red(pi_red_CAD)# delete
#robot.set_par_Dl(pi_dl_CAD)
#solve_dynamics(robot, config, print_table=True, export_file = True)
#solve_dynamics(robot, config, print_table=True, export_file = True, sigma_pi = metrics['parameters relative standard deviation'])
solve_dynamics(robot, config, robot_ground_truth = robot_gt, print_table=True, export_file = True, sigma_pi = metrics['parameters covariance matrix'])
# =========================================================================================================================== Plot REDUCED
# ==========================================================================================================================
hat_Pi =  np.hstack([robot.get_par_REG_red(),robot.get_par_Dl()]).reshape(-1,1)
metrics = get_metrics(robot,test_traject, hat_Pi=hat_Pi)
print('parameters relative standard deviation:')
print(metrics['parameters relative standard deviation'].T)

fig_identif = plot_identification(robot, test_traject, block = True)


def res_comparison(SOFTHAND=False):
    # Load softhand model
    MODEL_DYN = { "mass": 0.735522,                                     # EMPTY FRANKA (GOT FROM DH)
                "CoM_xyz":np.array([0.0105, -0.0043, -0.0454]),
                "ixx":0.0125,
                "ixy":-0.0004,
                "ixz":-0.0012,
                "iyy":0.01,
                "iyz":-0.0007,
                "izz":0.0048}
    
    # Load softhand model
    if SOFTHAND:
        MODEL_DYN = { "mass":1.25,                                        # SOFTHAND + link8 v3 (GOT FROM GAZEBO)
                    "CoM_xyz":np.array([0.005872, -0.000511, 0.149]),
                    "ixx":0.03,
                    "ixy":0.0,
                    "ixz":-0.0,
                    "iyy":0.03,
                    "iyz":0.0,
                    "izz":0.01}
    
    # MODEL_DYN = { "mass":1.024,                                      # ALLEGRO HAND (bad sim)
    #             "CoM_xyz":np.array([0.00, 0.0, 0.1475]),
    #             "ixx":1e-4,
    #             "ixy":0.0,
    #             "ixz":0.0,
    #             "iyy":1e-4,
    #             "iyz":0.0,
    #             "izz":1e-4}

    m_gt = MODEL_DYN['mass']
    com_gt = MODEL_DYN['CoM_xyz']
    Ib_gt = np.array([[MODEL_DYN['ixx'],MODEL_DYN['ixy'],MODEL_DYN['ixz']],
                    [MODEL_DYN['ixy'],MODEL_DYN['iyy'],MODEL_DYN['iyz']],
                    [MODEL_DYN['ixz'],MODEL_DYN['iyz'],MODEL_DYN['izz']]])

    l = 6 # unknwon link
    n = robot.numJoints

    # m_sh = FULL_DYN_PARAM_INITIAL_GUESS['mass'][l]
    # com_sh = np.array([FULL_DYN_PARAM_INITIAL_GUESS['CoM_x'][l], FULL_DYN_PARAM_INITIAL_GUESS['CoM_y'][l], FULL_DYN_PARAM_INITIAL_GUESS['CoM_z'][l]])
    # Ib_sh = np.array([[FULL_DYN_PARAM_INITIAL_GUESS['Ixx'][l], FULL_DYN_PARAM_INITIAL_GUESS['Ixy'][l], FULL_DYN_PARAM_INITIAL_GUESS['Ixz'][l]],
    #                 [FULL_DYN_PARAM_INITIAL_GUESS['Ixy'][l], FULL_DYN_PARAM_INITIAL_GUESS['Iyy'][l], FULL_DYN_PARAM_INITIAL_GUESS['Iyz'][l]],
    #                 [FULL_DYN_PARAM_INITIAL_GUESS['Ixz'][l], FULL_DYN_PARAM_INITIAL_GUESS['Iyz'][l], FULL_DYN_PARAM_INITIAL_GUESS['Izz'][l]]])
    # 2. Extract Estim values for Total lth-link
    params = robot.get_par_DYN().reshape((n, 10))
    m_estim = params[l, 0]                            # l-th link total mass
    if SOFTHAND:
        z_offset =  np.array([0,0, - 0.107])              # l-th link CoM w.r.t to frame-l (NOTE: DH frame is on the flange, 0.107 m shifted along z-axis)
    else:
        z_offset =  np.array([0,0,0])
    com_estim =  params[l, 1:4] - z_offset
    Ib_estim = (                                      # l-th inertia matrix w.r.t to barycentry frame
                np.array([[params[l,4], params[l,5], params[l,6]], 
                        [params[l,5], params[l,7], params[l,8]],
                        [params[l,6], params[l,8], params[l,9]]]) 
               )

    # 3. Prepare data for plotting
    com_labels = ['x', 'y', 'z']
    inertia_labels = ['Ixx', 'Iyy', 'Izz', 'Ixy', 'Ixz', 'Iyz']

    urdf_inertia = [Ib_gt[0,0], Ib_gt[1,1], Ib_gt[2,2], 
                    Ib_gt[0,1], Ib_gt[0,2], Ib_gt[1,2]]
    params_inertia = [Ib_estim[0,0], Ib_estim[1,1], Ib_estim[2,2], 
                    Ib_estim[0,1], Ib_estim[0,2], Ib_estim[1,2]]
    
    # 4. Create Subplots
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    if SOFTHAND:
        fig.suptitle("Estimation with SoftHand")
    else:
        fig.suptitle("Estimation without SoftHand")

    width = 0.35
    # Subplot A: Mass
    axes[0].bar(['Model', 'Estimation'], [m_gt, m_estim], color=['#3498db', '#e74c3c'], alpha=0.8)
    axes[0].set_title('Mass (kg)', fontweight='bold')
    axes[0].grid(axis='y', linestyle='--', alpha=0.6)
    # Subplot B: CoM
    x_com = np.arange(len(com_labels))
    axes[1].bar(x_com - width/2, com_gt, width, label='Model', color='#3498db')
    axes[1].bar(x_com + width/2, com_estim, width, label='Estimation', color='#e74c3c')
    axes[1].set_xticks(x_com)
    axes[1].set_xticklabels(com_labels)
    axes[1].set_title('Center of Mass (m)', fontweight='bold')
    axes[1].legend()
    axes[1].grid(axis='y', linestyle='--', alpha=0.6)
    # Subplot C: Inertia Elements
    x_ine = np.arange(len(inertia_labels))
    axes[2].bar(x_ine - width/2, urdf_inertia, width, label='Model', color='#3498db')
    axes[2].bar(x_ine + width/2, params_inertia, width, label='Estimation', color='#e74c3c')
    axes[2].set_xticks(x_ine)
    axes[2].set_xticklabels(inertia_labels)
    axes[2].set_title('Inertia (kg·m²)', fontweight='bold')
    axes[2].legend()
    axes[2].grid(axis='y', linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()
    return fig

#fig_comparison = res_comparison(SOFTHAND=False)
#fig_comparison = res_comparison(SOFTHAND=True)
fig_comparison = plot_link_solution(robot,robot_gt, 6)


# Store fil
# copy config file
import shutil
shutil.copy2(config_path, os.path.join(results_dir, os.path.basename(config_path)))
# save traject
fig_q.savefig(os.path.join(results_dir, 'q.png'), bbox_inches='tight', dpi=300)
fig_qd.savefig(os.path.join(results_dir, 'qd.png'), bbox_inches='tight', dpi=300)
fig_qdd.savefig(os.path.join(results_dir, 'qdd.png'), bbox_inches='tight', dpi=300)
fig_tau.savefig(os.path.join(results_dir, 'tau.png'), bbox_inches='tight', dpi=300)
# save fitting tau comparison
fig_fitting.savefig(os.path.join(results_dir, 'fitting_results.png'), bbox_inches='tight', dpi=300)
# save identification results
fig_identif.savefig(os.path.join(results_dir, 'identification_result.png'), bbox_inches='tight', dpi=300)
# Save last link comparison
fig_comparison.savefig(os.path.join(results_dir, 'comparison_7th_link.png'), bbox_inches='tight', dpi=300)