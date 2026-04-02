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
# Set up Franka Model ------
config_path = '/home/leo/Desktop/Base Inertial Parameter/src/config/franka_config.yaml'
with open(config_path, 'r') as f:
   config = yaml.load(f, Loader=SafeLoader)
franka = thunder.thunder_franka()
thunder.load_params(franka, config['robot']['path'])


from utils.identification_utils import print_base_inertial_parameters
print_base_inertial_parameters(franka)


# Unscented Transform ------------------------------------------------------------------------------------
def get_sigma_points(mu, var):
   N = mu.shape[0]
   alpha = 1e-3
   k = 0
   beta = 2
   lamb = alpha*alpha*(N+k)-N

   sigma = np.zeros((N,2*N+1))
   Weight_mean = np.zeros(2*N+1)
   Weight_cov = np.zeros(2*N+1)

   sigma[:,2*N] = mu
   Weight_mean[2*N] = lamb/(N+lamb)
   Weight_cov[2*N] = lamb/(N+lamb) + (1-alpha*alpha + beta)
   for i in range(N):
      sigma[:,i] = mu
      sigma[i,i] += np.sqrt((N+lamb)*var[i])
      sigma[:,N+i] = mu
      sigma[i,N+i] -= np.sqrt((N+lamb)*var[i])
      Weight_mean[i] = 1/(2*(N+lamb))
      Weight_cov[i] = 1/(2*(N+lamb))
      Weight_mean[N+i] = 1/(2*(N+lamb))
      Weight_cov[N+i] = 1/(2*(N+lamb))

   return sigma, (Weight_mean,Weight_cov)

franka_var = [
    # --- Link 1 ---
    0.04970684, 3.875e-4, 2.081e-4, 4.762e-3, 7.0337e-2, 7.0661e-2, 9.1170e-4, 1.3900e-5, 6.7720e-4, 1.9169e-3,
    # --- Link 2 ---
    0.00646926, 3.141e-4, 2.872e-3, 3.495e-4, 7.9620e-4, 2.8110e-03, 2.5995e-03, 3.9250e-4, 1.0254e-03, 7.0400e-05,
    # --- Link 3 ---
    0.03228604, 2.7518e-03, 3.9252e-03, 6.6502e-03, 3.7242e-03, 3.6155e-03, 1.0830e-03, 4.7610e-04, 1.1396e-03, 1.2805e-03,
    # --- Link 4 ---
    0.03587895, 5.317e-03, 1.04419e-02, 2.7454e-03, 2.5853e-03, 1.9552e-03, 2.8323e-03, 7.7960e-04, 1.3320e-04, 8.6410e-04,
    # --- Link 5 ---
    0.01225946, 1.1953e-03, 4.1065e-03, 3.8437e-03, 3.5549e-03, 2.9474e-03, 8.6270e-04, 2.1170e-04, 4.0370e-04, 2.2900e-05,
    # --- Link 6 ---
    0.01666555, 6.0149e-03, 1.4117e-03, 1.0517e-03, 1.9640e-04, 4.3540e-04, 5.4330e-04, 1.0900e-05, 1.1580e-04, 3.4100e-05,
    # --- Link 7 ---
    0.00735522, 1.0517e-03, 4.252e-04, 4.5403e-03, 1.2516e-03, 1.0027e-03, 4.8150e-04, 4.2800e-05, 1.1960e-04, 7.4100e-05,
]


sigma, (Weight_mean,Weight_cov) = get_sigma_points(
                                                      franka.get_par_DYN(),
                                                      franka_var
                                                   )

y = np.zeros((franka.get_par_REG_red().shape[0], sigma.shape[1]))
for i in range(sigma.shape[1]):
   # apply transform
   franka.set_par_DYN(sigma[:,i])
   franka.set_par_REG(franka.get_dyn2reg())
   y[:,i]=(franka.get_reg2red())

mu_y = y @ Weight_mean
Sigma_y = ((y - mu_y[:, None]) * Weight_cov) @ (y - mu_y[:, None]).T

print(mu_y)

# Montecarlo testing-------------------------------------------------------------------------------------------------
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

# 1. Setup the 70D Dynamic Prior
num_samples = 10000
mu_dyn = franka.get_par_DYN()
sigma_dyn = np.sqrt(franka_var) 
N_dyn = mu_dyn.shape[0]

# Generate 10,000 Gaussian samples of the full dynamic parameters
P_dyn_samples = np.random.normal(loc=mu_dyn, scale=sigma_dyn, size=(num_samples, N_dyn))

# 2. Determine the size of the Base Parameter vector (p_b)
franka.set_par_DYN(mu_dyn)
franka.set_par_REG(franka.get_dyn2reg())
pb_dim = franka.get_reg2red().shape[0]

P_b_samples = np.zeros((num_samples, pb_dim))

# 3. The End-to-End Mapping: p_dyn -> p_reg -> p_b
print(f"Mapping {num_samples} samples through the full robot dynamics...")
for i in range(num_samples):
    # Set the randomly sampled dynamic parameters
    franka.set_par_DYN(P_dyn_samples[i])
    # Apply the non-linear mapping g(p_dyn) to get the regressor parameters
    franka.set_par_REG(franka.get_dyn2reg())
    # Apply the linear projection to get the base parameters (p_b)
    P_b_samples[i] = franka.get_reg2red()

# 4. Calculate Empirical Statistics of p_b
mu_b = np.mean(P_b_samples, axis=0)
Sigma_b = np.cov(P_b_samples, rowvar=False)

# ---------------------------------------------------------
# TEST 1: Joint Multivariate Gaussianity of p_b
# ---------------------------------------------------------
# Use pseudo-inverse in case of rank deficiencies in the base parameter space
inv_Sigma_b = np.linalg.pinv(Sigma_b)
diff = P_b_samples - mu_b

# Squared Mahalanobis distance for the joint distribution
D2 = np.sum(diff @ inv_Sigma_b * diff, axis=1)

plt.figure(figsize=(14, 6))

# Plot 1: Chi-Squared Q-Q Plot for Joint Distribution
plt.subplot(1, 2, 1)
stats.probplot(D2, dist=stats.chi2(df=pb_dim), plot=plt)
plt.title(f"Joint Multivariate Normality of p_b\n(Chi-Squared Q-Q Plot, df={pb_dim})")
plt.xlabel("Theoretical Chi-Squared Quantiles")
plt.ylabel("Observed Mahalanobis D^2")
plt.tight_layout()
plt.show()

# # ---------------------------------------------------------
# # TEST 2: Marginal Gaussianity of a specific Base Parameter
# # ---------------------------------------------------------
# # Let's test an index that heavily relies on inertia or mass moments 
# # (You can change this index to inspect different base parameters)
param_to_test = 35 

plt.subplot(1, 2, 2)
stats.probplot(P_b_samples[:, param_to_test], dist="norm", plot=plt)
plt.title(f"Marginal Normality of p_b\n(Base Parameter Index {param_to_test})")
plt.xlabel("Theoretical Normal Quantiles")
plt.ylabel("Observed Parameter Value")

plt.tight_layout()
plt.show()

# ---------------------------------------------------------
# TEST 3: Formal Shapiro-Wilk Test on the Base Parameters
# ---------------------------------------------------------
print("\n--- Shapiro-Wilk Test on Marginal Base Parameters ---")
# Using 4999 samples as Shapiro-Wilk can become overly sensitive or error out with >5000
for j in range(pb_dim):
    stat, p_val = stats.shapiro(P_b_samples[:4999, j]) 
    
    # Check for skewness and kurtosis to understand WHY it fails
    skewness = stats.skew(P_b_samples[:, j])
    kurtosis = stats.kurtosis(P_b_samples[:, j]) # Fisher definition (normal = 0)
    
    is_gauss = "PASS" if p_val > 0.05 else "FAIL"
    print(f"p_b[{j}]: {is_gauss} (p={p_val:.4f}) | Skew: {skewness:.2f}, Kurtosis: {kurtosis:.2f}")






exit()

#--------------------------------------------------------------------------------------------------------------------
# Set up Thumb Model ------------------------------------------------------------------------------------------------
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



