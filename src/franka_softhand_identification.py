"""Estimate reduced inertial parameters from ROS2 bag joint data.

This script reads joint states from a rosbag, filters the signals,
computes the dynamic regressor for a reduced parameter set, and
estimates parameters by least-squares. It also computes simple
friction terms (viscous + smooth static) and plots results.

The code relies on helper functions and classes from `utils`.
"""
import utils.import_thunder as thunder

import numpy as np
import matplotlib.pyplot as plt

from utils.loader_utils import *
from utils.identification_utils import *
from utils.evaluation_utils import *


# 1. Define and create the results directory
date_str = datetime.datetime.now().strftime("%d_%m_%Y")
results_dir = f"results/data_{date_str}/franka_softhand"
if not os.path.exists(results_dir):
        os.makedirs(results_dir)
# ---------------------------------------------------


np.random.seed(41)
np.set_printoptions(precision=4, suppress=True, linewidth=200)

# Franka ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/franka_softhand_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
robot = thunder.thunder_franka()
robot_ig = thunder.thunder_franka()
robot_gt = thunder.thunder_franka()

# load robot params
config_robot = config['robot']
try:
    # Set initial guess
    thunder.load_params(robot, config_robot['path'])
    thunder.load_params(robot_ig, config_robot['path'])

    # Set ground truth
    gt_params_path = "/home/leo/Desktop/Base Inertial Parameter/src/thunder/franka_generatedFiles/param/franka_SH_par.yaml"
    thunder.load_params(robot_gt, gt_params_path)
    pi_red_CAD = robot_gt.get_par_REG_red()
    pi_dl_CAD = robot_gt.get_par_Dl()
    # Add initial guess with noise
    #std = 0.1
    std = 0
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
test_traject = TrajectoryManager(config_path, config_processing['butterworth_order'],config_processing['cut_off_frequency'], config_processing['decimate_factor'])
test_traject.df = TrajectoryManager.read_bag(config_traject['bag'], config_traject['topic'], config_traject['joints'])


config_traject_interval = config_traject.get('interval', (5,100))
apply_decimation = config_processing.get('apply_decimation', False)
test_traject.process(interval_min=config_traject_interval[0], interval_max=config_traject_interval[1], apply_decimation=apply_decimation)



fig_q, fig_qd, fig_qdd, fig_tau = test_traject.plot_traject(robot, block=False)


# Base inertial parameters
print("\n>> Base Inertial Parametersc Identification")
print("Base Inertial Parameters:")
bp = print_base_inertial_parameters(robot)
# =========================================================================================================================== Compute regressor
# ==========================================================================================================================
YY, TTau = get_big_Y_Tau(robot, test_traject)

n = robot.numJoints
M = len(test_traject.t)
p = YY.shape[1]

print(f"\n> Identification via {config_iden['method']} method:")
print("Number of samples (M): ", M)
print("Number of parameters in the base: ", p)
print("Shape YY (Mnxp): ", YY.shape)
print("Shape TTau (Mnx1): ", TTau.shape)


# =========================================================================================================================== Compute Metrics
# ==========================================================================================================================

conditioning_ratio = config_iden.get('conditioning_ratio', None)

# Method 1 - OLS ---------------------------------------------------------------
hat_pi, metrics = solve_OLS(robot, test_traject, conditioning_ratio = conditioning_ratio)
if config_iden['method'] == 'OLS|prior':
    hat_pi, metrics = solve_OLS_with_prior(robot, test_traject, conditioning_ratio = conditioning_ratio)


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


# Compute Essential Parameters  ---------------------------------------------------------------
compute_SVD_essential(robot, test_traject, conditioning_ratio = config_iden.get('conditioning_ratio', 30))
hat_pi_e, idx_e = compute_essential(robot, test_traject, ratio_essential = config_iden.get('ratio_essential', 30))


print(f"Essential parameters indexes: {idx_e}, {len(idx_e)} out of {p} (ratio > 30)")
print(f"Essential parameters Conditinoning: {np.linalg.cond(YY[:,idx_e])}")

# Method 2 - weighted LS ---------------------------------------------------------------
print("\nComputing reduced pi solution by WLS")
hat_pi_OLS = hat_pi.copy()

if config_iden['method'] == 'WLS':
    hat_pi, metrics = solve_WLS(robot, test_traject, conditioning_ratio = conditioning_ratio)
elif config_iden['method'] == 'WLS|prior':
    hat_pi, metrics = solve_WLS_with_prior(robot, test_traject, conditioning_ratio = conditioning_ratio)

# Assign estimation of base inertial parameters
hat_pi_REG_red = hat_pi[:-n*(robot.Dl_order)]
hat_pi_dl = hat_pi[-n*(robot.Dl_order):]
robot.set_par_REG_red(hat_pi_REG_red)
robot.set_par_Dl(hat_pi_dl)



print("\n> Results:")
# Compute residual norm and singular values for analysis
Pi_CAD = np.hstack([pi_red_CAD, pi_dl_CAD]).reshape(-1,1)
residual = np.linalg.norm(TTau - YY @ Pi_CAD)
print(f"Residuals (CAD): {residual} [Nm]" )

residual = np.linalg.norm(TTau - YY @ hat_pi_OLS)
print(f"Residuals (OLS): {residual} [Nm]" )

residual = np.linalg.norm(TTau - YY @ hat_pi)
print(f"Residuals: {residual} [Nm]" )

print("Parameters estimation")
print(hat_pi.flatten())
print(f"error std. deviation: {sigma_w}")
print("Parameters std. deviation:")
print(sigma_pi.flatten())
print("Parameters std. deviation (%):")
print(sigma_pi_perc.flatten())

fig_base = plot_eval_LS_solution(hat_pi, metrics, robot_gt = robot_gt, block = False)

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
    delta_tau_OLS.append(test_traject.tau[i,:] - (Y_red@hat_pi_OLS[:-n*(robot.Dl_order)] + Y_dl@hat_pi_OLS[-n*(robot.Dl_order):]).flatten())
    delta_tau_E.append(test_traject.tau[i,:] - (Y_[:,idx_e] @ hat_pi_e).flatten())
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


# --- 1. Plot tau ---
cols = int(np.sqrt(n)) 
rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
JOINT_NAMES = config_traject['joints']
fig_fitting = plt.figure(figsize=(12,8))
for i in range(n):
    plt.subplot(rows, cols,i+1)
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
plt.show(block = False)

# =========================================================================================================================== CasADi Solver
# ==========================================================================================================================
print("\n>> Full Dynamic Identification")
sol = solve_dynamics(robot, config, sigma_pi = metrics['parameters covariance matrix'], export_file = True, path = results_dir)
results = sol[-1] # minimization values
# Assign estimation
robot.set_par_REG(sol[0])
# Convert into dynamic and regressor parameterss
robot.set_par_DYN(robot.get_reg2dyn())
robot.set_par_REG_red(robot.get_reg2red())

plot_table(robot, robot_ground_truth = robot_gt)


# =========================================================================================================================== Plot REDUCED
# ==========================================================================================================================
fig_identif = plot_identification(robot, test_traject, metrics, block = False)
fig_comparison = plot_link_solution(robot, robot_gt, 6, block = True)


# =========================================================================================================================== Store Results
# ==========================================================================================================================
print("\n>> Export:")
# copy config file
import shutil
shutil.copy2(config_path, os.path.join(results_dir, os.path.basename(config_path)))
# save traject
fig_q.savefig(os.path.join(results_dir, 'q.png'), bbox_inches='tight', dpi=300)
fig_qd.savefig(os.path.join(results_dir, 'qd.png'), bbox_inches='tight', dpi=300)
fig_qdd.savefig(os.path.join(results_dir, 'qdd.png'), bbox_inches='tight', dpi=300)
fig_tau.savefig(os.path.join(results_dir, 'tau.png'), bbox_inches='tight', dpi=300)
# save base param estim tau comparison
fig_base.savefig(os.path.join(results_dir, 'base_param_results.png'), bbox_inches='tight', dpi=300)
# save fitting tau comparison
fig_fitting.savefig(os.path.join(results_dir, 'fitting_results.png'), bbox_inches='tight', dpi=300)
# save identification results
fig_identif.savefig(os.path.join(results_dir, 'identification_result.png'), bbox_inches='tight', dpi=300)
# Save last link comparison
fig_comparison.savefig(os.path.join(results_dir, 'comparison_7th_link.png'), bbox_inches='tight', dpi=300)