"""
utilities function for computation of the idientification process.
"""

import os
import datetime
import numpy as np
import yaml

import matplotlib.pyplot as plt

import casadi
from utils.casadi_utils import *


def print_base_inertial_parameters(robot):
    param_names = ['M','MX','MY','MZ','XX','XY','XZ','YY','YZ','ZZ','Ia']

    beta = robot.get_beta()
    n = robot.numJoints
    parPerLink = robot.STD_PAR_LINK
    description = ""
    description_dict = {}

    for i in range(beta.shape[0]):
        line_terms = []
        for j in range(beta.shape[1]):
            coeff = beta[i, j]
            if coeff == 0:
                continue

            # Identify the parameter name
            if j >= n * parPerLink:
                p_name = f"{param_names[10]}_{j - n * parPerLink}"
            else:
                p_name = f"{param_names[j % parPerLink]}_{j // parPerLink}"

            # Formatting the term
            if abs(coeff) == 1.0:
                formatted_term = p_name
            else:
                formatted_term = f"{abs(coeff):.2e} * {p_name}"

            # Manage the sign and joining
            if not line_terms:
                # First term of the line: only add '-' if negative
                prefix = "-" if coeff < 0 else ""
                line_terms.append(f"{prefix}{formatted_term}")
            else:
                # Subsequent terms: always add ' + ' or ' - '
                op = " - " if coeff < 0 else " + "
                line_terms.append(f"{op}{formatted_term}")

        description += "".join(line_terms) + "\n"
        description_dict[f"{i}"] = line_terms

    print(description)
    return description_dict

def get_big_Y_Tau(robot, traject):
    ## YY oredred for [reg_red, friction]
    dt = np.mean(np.diff(traject.t))

    Y_log = []
    for i in range(traject.t.shape[0]):
        q = traject.q[i,:]
        dq = traject.qd[i,:]
        ddq = traject.qdd[i,:]

        robot.set_q(q)
        robot.set_dq(dq)
        robot.set_ddq(ddq)
        robot.set_dqr(dq)
        robot.set_ddqr(ddq)

        # Full inverse-dynamics regressor (per-joint rows)
        Y_red = robot.get_Yr_red()
        Y_dl = robot.get_reg_dl()
        Y = np.hstack([Y_red, Y_dl])
        Y_log.append(Y)

    # Stack regressors and torques into the big LS problem
    M = len(traject.t)
    n = robot.numJoints
    YY = np.concatenate([np.asarray(Y) for Y in Y_log], axis=0)  # (M*n) x p
    TTau = np.concatenate([np.asarray(t).reshape(n, 1) for t in traject.tau], axis=0)  # (M*n) x 1

    return YY, TTau

def solve_OLS(robot, traject, conditioning_ratio = None):
    # > Compute pi_OLS = (Y.t @ Y)^-1 @ Y.t @ tau
    YY, TTau = get_big_Y_Tau(robot, traject)

    if np.linalg.matrix_rank(YY)<YY.shape[1]:
        print(f"[WARNING] Matrix is not full colum rank {np.linalg.matrix_rank(YY)}/{YY.shape[1]} - the solution is not unique" )

    p = YY.shape[1] + 1 # mininum eigenvalue
    if conditioning_ratio is not None:
        U,S,Vh = np.linalg.svd(YY.T @ YY,  compute_uv = True)
        sigma_max = S[0]
        for i in range(1,YY.shape[1]):
            sigma_p = S[i]
            cond = np.sqrt(sigma_max/sigma_p)
            if cond > conditioning_ratio:
                print(f"OLS: From now on (index {i}), high ill-conditioning")
                p = i
                break


        Sinv_trunc = np.linalg.inv(np.diag(S))
        Sinv_trunc[p:,p:] = 0
        pseudo_inv = U @ Sinv_trunc @ Vh
    else:
        pseudo_inv =  np.linalg.inv(YY.T @ YY)

    hat_pi = pseudo_inv @ YY.T @ TTau

    # > Compute Solution metrics
    metrics = {}
    metrics['conditioning number'] = np.linalg.cond(YY)

    residual = TTau - YY @ hat_pi
    metrics['residual'] = np.linalg.norm(residual)

    # std.dev. of residual error
    n = YY.shape[0]
    p = hat_pi.shape[0]
    sigma_w_2 = np.linalg.norm((TTau - YY@hat_pi),ord=2)**2 / (
        n - p
    )
    metrics['error standard deviation'] = sigma_w_2

    # covariance matrix of estimated parameters
    C_w = sigma_w_2 * pseudo_inv                # <-- Check
    # C_w = sigma_w_2 * np.linalg.inv(YY.T@YY)  # <-- Check
    metrics['parameters covariance matrix'] = C_w

    # relative std.dev. of estimated parameters
    sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    metrics['parameters standard deviation'] = sigma_pi
    metrics['parameters relative standard deviation'] = sigma_pi_perc


    return hat_pi, metrics

def solve_OLS_with_prior(robot, traject, conditioning_ratio = None):
    # > Compute pi_OLS = (Y.t @ Y)^-1 @ Y.t @ tau
    hat_pi_ref = np.hstack([robot.get_par_REG_red(), robot.get_par_Dl()]).reshape(-1,1)
    YY, TTau = get_big_Y_Tau(robot, traject)

    if conditioning_ratio is not None:
        U,S,Vh = np.linalg.svd(YY.T @ YY,  compute_uv = True)
        sigma_max = S[0]
        for i in range(1,YY.shape[1]):
            sigma_p = S[i]
            cond = np.sqrt(sigma_max/sigma_p)
            if cond > conditioning_ratio:
                print(f"OLS: From now on (index {i}), high ill-conditioning")
                p = i
                break
        S_trunc = np.linalg.inv(np.diag(S))
        S_trunc[p:,p:] = 0
        pseudo_inv = U @ S_trunc @ Vh
    else:
        pseudo_inv =  np.linalg.inv(YY.T @ YY)

    hat_pi = hat_pi_ref + pseudo_inv @ YY.T @ (TTau - YY @ hat_pi_ref)

    # > Compute metrics
    metrics = {}
    metrics['conditioning number'] = np.linalg.cond(YY)

    residual = TTau - YY @ hat_pi
    metrics['residual'] = np.linalg.norm(residual)

    # std.dev. of residual error
    n = YY.shape[0]
    p = hat_pi.shape[0]
    sigma_w_2 = np.linalg.norm((TTau - YY@hat_pi),ord=2)**2 / (
        n - p
    )
    metrics['error standard deviation'] = sigma_w_2

    # covariance matrix of estimated parameters
    C_w = sigma_w_2 * pseudo_inv                # <-- Check
    # C_w = sigma_w_2 * np.linalg.inv(YY.T@YY)  # <-- Check
    metrics['parameters covariance matrix'] = C_w

    # relative std.dev. of estimated parameters
    sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    metrics['parameters standard deviation'] = sigma_pi
    metrics['parameters relative standard deviation'] = sigma_pi_perc

    return hat_pi, metrics

def solve_WLS(robot, traject, conditioning_ratio = None):
    """
    Weight differently each joint,
      Solve:
        W @ Tau_w = W @ Y @ Pi

    with W = blkdiag( diag(rho1, ..., rhon), diag(rho1, ..., rhon), ..., diag(rho1, ..., rhon)) in NbxNb
    """

    # > Compute pi_WLS = (Y.t @ G.t @ G @ Y)^-1 @ Y.t @ G.t @ tau
    # OLS solution for residuals
    YY, TTau = get_big_Y_Tau(robot, traject)
    hat_pi, _ = solve_OLS(robot, traject, conditioning_ratio=conditioning_ratio)
    # computation of weighting matrix
    sigma_w_2 = np.zeros(robot.numJoints)   # effort measurements noise std.dev. at joint level
    nb_samples = YY.shape[0]            # num of samples
    p = YY.shape[1]                     # num of parameters
    W_diag = np.zeros(nb_samples*robot.numJoints)

    # pre-compute all effort residual
    residual = TTau - YY @ hat_pi

    # std.dev for joint i-th measurements using OLS residuals
    for i in range(robot.numJoints):
        diff = residual[i::robot.numJoints]
        denom = nb_samples - p
        sigma_w_2[i] = np.sqrt(np.linalg.norm(diff)**2 / denom)

        # compute weight (inverse of std.dev) of joint i-th
        for j in range(nb_samples):
            W_diag[j + i*robot.numJoints] = 1 / sigma_w_2[i]

    # compute weights multiplication for each joint
    YY_weighted = np.zeros_like(YY)
    TTau_weighted = np.zeros_like(TTau)
    for i in range(robot.numJoints):
        YY_weighted[i::robot.numJoints,:] = W_diag[i]*YY[i::robot.numJoints, :] # apply weight for ith raw
        TTau_weighted[i::robot.numJoints] = W_diag[i]*TTau[i::robot.numJoints]  # apply weight for ith value

    # Compute inverted matrix and solution
    if conditioning_ratio is not None:
        r = YY.shape[1] + 1                 # index of last singular value
        U,S,Vh = np.linalg.svd(YY_weighted.T @ YY_weighted,  compute_uv = True)
        sigma_max = S[0]
        for i in range(1,YY_weighted.shape[1]):
            sigma_p = S[i]
            cond = np.sqrt(sigma_max/sigma_p)
            if cond > conditioning_ratio:
                print(f"WLS: From now on (index {i}), high ill-conditioning")
                r = i
                break

        S_trunc = np.linalg.inv(np.diag(S))
        S_trunc[r:,r:] = 0
        pseudo_inv = U @ S_trunc @ Vh
    else:
        pseudo_inv = np.linalg.inv(YY_weighted.T @ YY_weighted)
    hat_pi = pseudo_inv @ YY_weighted.T @ TTau_weighted

    # > Compute metrics
    metrics = {}
    metrics['conditioning number'] = np.linalg.cond(YY)

    residual = TTau_weighted - YY_weighted @ hat_pi
    metrics['residual'] = np.linalg.norm(residual)

    # std.dev. of residual error
    n = YY.shape[0]
    p = hat_pi.shape[0]
    sigma_w_2 = np.linalg.norm((TTau_weighted - YY_weighted@hat_pi),ord=2)**2 / (
        n - p
    )
    metrics['error standard deviation'] = sigma_w_2

    # covariance matrix of estimated parameters
    C_w = sigma_w_2 * pseudo_inv                # <-- Check
    # C_w = sigma_w_2 * np.linalg.inv(YY.T@YY)  # <-- Check
    metrics['parameters covariance matrix'] = C_w

    # relative std.dev. of estimated parameters
    sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    metrics['parameters standard deviation'] = sigma_pi
    metrics['parameters relative standard deviation'] = sigma_pi_perc

    return hat_pi, metrics

def solve_WLS_with_prior(robot, traject, conditioning_ratio = 50):
    """
    Weight differently each joint,
      Solve:
        W @ Tau_w = W @ Y @ Pi
    """
    # Compute pi_WLS = (Y.t @ G.t @ G @ Y)^-1 @ Y.t @ G.t @ tau

    # OLS solution for residuals
    hat_pi_ref = np.hstack([robot.get_par_REG_red(),robot.get_par_Dl()]).reshape(-1,1)
    YY, TTau = get_big_Y_Tau(robot, traject)
    hat_pi, _ = solve_OLS_with_prior(robot, traject, conditioning_ratio=conditioning_ratio)


    # computation of weighting matrix
    sigma_w_2 = np.zeros(robot.numJoints)   # effort measurements noise std.dev. at joint level
    nb_samples = YY.shape[0]            # num of samples
    p = YY.shape[1]                     # num of parameters
    W_diag = np.zeros(nb_samples*robot.numJoints)

    residual = TTau - YY @ hat_pi
    for i in range(robot.numJoints):

        # std.dev for joint i-th measuerements using OLS residuals
        diff = residual[i::robot.numJoints]
        denom = nb_samples - p
        sigma_w_2[i] = np.sqrt(np.linalg.norm(diff)**2 / denom)

        # assign inverse for all samples of joint i-th
        for j in range(nb_samples):
            W_diag[j + i*robot.numJoints] = 1 / sigma_w_2[i]

    # compute weights multiplication for each joint
    YY_weighted = np.zeros_like(YY)
    TTau_weighted = np.zeros_like(TTau)
    for i in range(robot.numJoints):
        YY_weighted[i::robot.numJoints,:] = W_diag[i]*YY[i::robot.numJoints, :] # apply weight for ith raw
        TTau_weighted[i::robot.numJoints] = W_diag[i]*TTau[i::robot.numJoints]  # apply weight for ith value

    # Compute inverted matrix and solution
    if conditioning_ratio is not None:
        r = YY.shape[1] + 1                 # num of parameters
        U,S,Vh = np.linalg.svd(YY_weighted.T @ YY_weighted,  compute_uv = True)
        sigma_max = S[0]
        for i in range(YY_weighted.shape[1]):
            sigma_p = S[i]
            cond = np.sqrt(sigma_max/sigma_p)
            if cond > conditioning_ratio:
                print(f"WLS: From now on (index {i}), high ill-conditioning")
                r = i
                break

        S_trunc = np.linalg.inv(np.diag(S))
        S_trunc[r:,r:] = 0
        pseudo_inv = U @ S_trunc @ Vh
        hat_pi = hat_pi_ref + pseudo_inv @ YY.T @ (TTau - YY @ hat_pi_ref)
    else:
        hat_pi = hat_pi_ref +  np.linalg.inv(YY.T @ YY) @ YY.T @ (TTau - YY @ hat_pi_ref)

    # > Compute metrics
    metrics = {}
    metrics['conditioning number'] = np.linalg.cond(YY)

    residual = TTau_weighted - YY_weighted @ hat_pi
    metrics['residual'] = np.linalg.norm(residual)

    # std.dev. of residual error
    n = YY.shape[0]
    p = hat_pi.shape[0]
    sigma_w_2 = np.linalg.norm((TTau_weighted - YY_weighted@hat_pi),ord=2)**2 / (
        n - p
    )
    metrics['error standard deviation'] = sigma_w_2

    # covariance matrix of estimated parameters
    C_w = sigma_w_2 * pseudo_inv                # <-- Check
    # C_w = sigma_w_2 * np.linalg.inv(YY.T@YY)  # <-- Check
    metrics['parameters covariance matrix'] = C_w

    # relative std.dev. of estimated parameters
    sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    metrics['parameters standard deviation'] = sigma_pi
    metrics['parameters relative standard deviation'] = sigma_pi_perc

    return hat_pi, metrics



# def get_metrics(robot, traject, hat_pi = None, hat_pi_essential = None, idx_essental = None):
#     YY, TTau = get_big_Y_Tau(robot, traject)
#     metrics = {}
#     metrics['conditioning number'] = np.linalg.cond(YY)

#     if hat_pi is None and hat_pi_essential is None:
#         # Ordinary Leat Square solution
#         hat_pi = np.linalg.pinv(YY) @ TTau
#     elif hat_pi is None and hat_pi_essential is not None:
#         hat_pi = hat_pi_essential
#         YY = YY[:,idx_essental]

#     residual = TTau - YY @ hat_pi
#     metrics['residual'] = np.linalg.norm(residual)

#     # std.dev. of residual error
#     n = YY.shape[0]
#     p = hat_pi.shape[0]
#     sigma_w_2 = np.linalg.norm((TTau - YY@hat_pi),ord=2)**2 / (
#         n - p
#     )
#     metrics['error standard deviation'] = sigma_w_2

#     # covariance matrix of estimated parameters
#     C_w = sigma_w_2 * np.linalg.inv(YY.T@YY)
#     metrics['parameters covariance matrix'] = C_w

#     # relative std.dev. of estimated parameters
#     sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
#     sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
#     metrics['parameters standard deviation'] = sigma_pi
#     metrics['parameters relative standard deviation'] = sigma_pi_perc

#     return metrics

def compute_essential(robot, traject, ratio_essential = 30):
    #
    # ratio ideally in 0-30
    YY, TTau = get_big_Y_Tau(robot, traject)
    mask_essential = np.ones((YY.shape[1]), dtype=bool) # mask of essential parameters
    indeces_essential = [i for i in range(YY.shape[1]) if mask_essential[i]]

    #
    print(f"Computing essential parameters with ratio {ratio_essential} between max and min relative std.dev.")
    hat_pi = np.linalg.pinv(YY) @ TTau
    n = YY.shape[0]
    p = hat_pi.shape[0]
    sigma_w_2 = np.linalg.norm((TTau - np.dot(YY, hat_pi)),ord=2)**2 / (
        n - p
    )
    C_w = sigma_w_2 * np.linalg.inv(np.dot(YY.T, YY))
    sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
    s_max, idx_max = np.max(sigma_pi_perc), np.argmax(sigma_pi_perc)
    s_min, idx_min = np.min(sigma_pi_perc), np.argmin(sigma_pi_perc)

    while s_max/s_min > ratio_essential:
        index_to_remove = indeces_essential[idx_max]
        mask_essential[index_to_remove] = False
        indeces_essential = [i for i in range(YY.shape[1]) if mask_essential[i]]

        YY_e = YY[:, indeces_essential]

        hat_pi = np.linalg.pinv(YY_e) @ TTau
        n = YY_e.shape[0]
        p = hat_pi.shape[0]
        sigma_w_2 = np.linalg.norm((TTau - np.dot(YY_e, hat_pi)),ord=2)**2 / (
            n - p
        )
        C_w = sigma_w_2 * np.linalg.inv(np.dot(YY_e.T, YY_e))
        sigma_pi = np.array([np.sqrt(C_w[i,i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
        sigma_pi_perc = np.array([100*sigma_pi[i]/np.abs(hat_pi[i]) for i in range(hat_pi.shape[0])]).reshape(hat_pi.shape)
        s_max, idx_max = np.max(sigma_pi_perc), np.argmax(sigma_pi_perc)
        s_min, idx_min = np.min(sigma_pi_perc), np.argmin(sigma_pi_perc)

    hat_pi_essential = hat_pi

    return hat_pi_essential, indeces_essential

def compute_SVD_essential(robot, traject, conditioning_ratio = 50):

    YY, _ = get_big_Y_Tau(robot, traject)

    U,S,Vh = np.linalg.svd(YY.T @ YY,  compute_uv = True)
    sigma_max = S[0]
    for i in range(YY.shape[1]):
        sigma_p = S[i]
        cond = np.sqrt(sigma_max/sigma_p)
        if cond > conditioning_ratio:
            print(f"SVD: truncated at index {i}")
            p = i
            break

    most_important_along_svd_dir = []
    for i in range(p):
        most_important_along_svd_dir.append(np.argmax(np.abs(Vh[i,:])))

    return Vh[:p]

def solve_dynamics(robot, config, sigma_pi = None, export_file = False, path = None):
    # robot.set_par_REG_red(...) must be called before to set proper estimated base inertial parameters
    POSITIVE_THRESH = float(config['identification'].get('positive_threshold',1e-16))

    FULL_DYN_PARAM_INITIAL_GUESS = {}
    FULL_DYN_PARAM_INITIAL_GUESS['mass']  = robot.get_par_DYN()[0::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['CoM_x'] = robot.get_par_DYN()[1::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['CoM_y'] = robot.get_par_DYN()[2::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['CoM_z'] = robot.get_par_DYN()[3::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Ixx']   = robot.get_par_DYN()[4::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Ixy']   = robot.get_par_DYN()[5::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Ixz']   = robot.get_par_DYN()[6::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Iyy']   = robot.get_par_DYN()[7::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Iyz']   = robot.get_par_DYN()[8::10].copy()
    FULL_DYN_PARAM_INITIAL_GUESS['Izz']   = robot.get_par_DYN()[9::10].copy()
    if hasattr(robot,"get_par_Ia"):
        par_per_link = robot.STD_PAR_LINK + 1
    else:
        par_per_link = robot.STD_PAR_LINK
        print(f"[INFO]: the model used does not include motor inertia" )

    # init guess on urdf values for inertial values. Convert from dynamic to regressor
    n = robot.numJoints
    hat_par_REG_red_star = robot.get_par_REG_red().copy() # Extract reduced estimated base inertial parameters via LS 

    # load init guess regressor
    hat_par_DYN_0 = np.zeros(n*par_per_link)    # Set initial guess for full regressor parameters 
    hat_par_REG_0 = np.zeros(n*par_per_link)
    for i in range(n):
        hat_par_DYN_0[i*10] = FULL_DYN_PARAM_INITIAL_GUESS['mass'][i]
        hat_par_DYN_0[i*10+1] = FULL_DYN_PARAM_INITIAL_GUESS['CoM_x'][i]
        hat_par_DYN_0[i*10+2] = FULL_DYN_PARAM_INITIAL_GUESS['CoM_y'][i]
        hat_par_DYN_0[i*10+3] = FULL_DYN_PARAM_INITIAL_GUESS['CoM_z'][i]
        hat_par_DYN_0[i*10+4] = FULL_DYN_PARAM_INITIAL_GUESS['Ixx'][i]
        hat_par_DYN_0[i*10+5] = FULL_DYN_PARAM_INITIAL_GUESS['Ixy'][i]
        hat_par_DYN_0[i*10+6] = FULL_DYN_PARAM_INITIAL_GUESS['Ixz'][i]
        hat_par_DYN_0[i*10+7] = FULL_DYN_PARAM_INITIAL_GUESS['Iyy'][i]
        hat_par_DYN_0[i*10+8] = FULL_DYN_PARAM_INITIAL_GUESS['Iyz'][i]
        hat_par_DYN_0[i*10+9] = FULL_DYN_PARAM_INITIAL_GUESS['Izz'][i]

    # convert CAD parameters into regressor parameters
    robot.set_par_DYN(hat_par_DYN_0[:n*10])
    hat_par_REG_0[:len(robot.get_dyn2reg())] = robot.get_dyn2reg()                          # regressor
    if (par_per_link == 11): hat_par_REG_0[len(robot.get_dyn2reg()):] = robot.get_par_Ia()  # motor inertia

    # define optim problem
    hat_par_REG_sym = casadi.SX.sym('Pi', n*par_per_link)    # Parameters to estimate (dyn + motor inertia)


    g = []          # Constraints
    ub = []         # Upper bound
    lb = []         # Lower bound
    d = casadi.SX(n,5)
    for i in range(n):
        m = hat_par_REG_sym[i*10]                     # mass

        # Mass must be positive
        g += [m]             # [Kg]
        lb += [1e-3]
        ub += [casadi.inf]

        # positive definite inertia matrix
        Ib_reg = casadi.SX(3,3)
        Ib_reg[0,0] = hat_par_REG_sym[i*10 + 4]
        Ib_reg[0,1] = hat_par_REG_sym[i*10 + 5]
        Ib_reg[0,2] = hat_par_REG_sym[i*10 + 6]
        Ib_reg[1,0] = hat_par_REG_sym[i*10 + 5]
        Ib_reg[1,1] = hat_par_REG_sym[i*10 + 7]
        Ib_reg[1,2] = hat_par_REG_sym[i*10 + 8]
        Ib_reg[2,0] = hat_par_REG_sym[i*10 + 6]
        Ib_reg[2,1] = hat_par_REG_sym[i*10 + 8]
        Ib_reg[2,2] = hat_par_REG_sym[i*10 + 9]
        
        c = casadi.SX(3,1)
        c[0,0] = hat_par_REG_sym[i*10 + 1]/(m+POSITIVE_THRESH)
        c[1,0] = hat_par_REG_sym[i*10 + 2]/(m+POSITIVE_THRESH)
        c[2,0] = hat_par_REG_sym[i*10 + 3]/(m+POSITIVE_THRESH)
        
        Ib = Ib_reg - m @casadi_skew(c).T @ casadi_skew(c)

        # !Criterio Sylvestr: det sottomatrici!
        d[i,0] = det(Ib[:3,:3])
        d[i,1] = det(Ib[:2,:2])
        d[i,2] = det(Ib[:1,:1])
        d[i,3] = det(Ib[2,2])
        d[i,4] = det(Ib[1,1])
        g +=  [d[i,0],              d[i,1],                 d[i,2], d[i,3], d[i,4]]
        lb += [POSITIVE_THRESH,     POSITIVE_THRESH,        POSITIVE_THRESH, POSITIVE_THRESH, POSITIVE_THRESH]
        ub += [casadi.inf,          casadi.inf,             casadi.inf, casadi.inf, casadi.inf]

        # Positive motor inertias
        if par_per_link > 10:
            Ia = hat_par_REG_sym[n*10 + i]
            g +=  [Ia]
            lb += [POSITIVE_THRESH]
            ub += [casadi.inf]

    # Define Optim Cost
    config_ident = config['identification']
    w_loss    = float(config_ident['weight']['loss'])               # Pull toward base parameters
    w_mass    = float(config_ident['weight']['mass'])               # Pull toward URDF mass
    w_com     = float(config_ident['weight']['CoM'])                # Pull toward URDF CoM
    w_inertia = float(config_ident['weight']['inertia'])            # Pull toward URDF inertia
    w_link = np.array(config_ident['weight']['link'], dtype=float)   # wheight link differently
    if len(w_link)!=n:
        print("[WARN] The lenght of link weight is bad setted. proceding with equal weighting for each link")
        w_link = [1 for i in range(n)]
    w_link = w_link/np.sum(w_link)

    # resid beta
    sigma_pi_inv = 0
    if sigma_pi is not None and sigma_pi.shape[0] != sigma_pi.shape[0]:
        # weight with inverse of relative std.dev.
        sigma_pi_inv = np.diag(1/sigma_pi.flatten())
        W_sigma_inv = sigma_pi_inv/np.trace(sigma_pi_inv) # sigma_pi is parameters relative std.dev
        f_loss = (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym).T @ W_sigma_inv[:-n*2,:-n*2] @ (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym)
    elif sigma_pi is not None and sigma_pi.shape[0] != sigma_pi.shape[0]:
        # use covariance matrix
        W_sigma_inv = np.linalg.inv(sigma_pi)
        f_loss = (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym).T @ W_sigma_inv @ (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym)
    else:
        # do not use weights
        f_loss = (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym).T @ (hat_par_REG_red_star - robot.get_beta()@hat_par_REG_sym)

    # additional weights
    f_mass = 0
    f_com = 0
    f_inertia = 0
    a = np.min(hat_par_REG_0[0::10]) # M
    b = np.min(np.hstack([hat_par_REG_0[1::10],hat_par_REG_0[2::10],hat_par_REG_0[3::10]])) # MCoM
    c = np.min(np.hstack([hat_par_REG_0[4::10],hat_par_REG_0[7::10],hat_par_REG_0[9::10]])) # I on diagonal
    w_mass    = 1/w_mass               # define inv. of std. dev
    w_com     = 1/w_com
    w_inertia = 1/w_inertia
    for i in range(n):
        par_REG_link_i = hat_par_REG_0[10*i:10*(i+1)]

        f_mass      +=  w_link[i] @  casadi.sumsqr(hat_par_REG_sym[10*i] - par_REG_link_i[0]) # M
        f_com       +=  w_link[i] @ (casadi.sumsqr(hat_par_REG_sym[10*i+1]/hat_par_REG_sym[10*i] - par_REG_link_i[1]/par_REG_link_i[0]) + # CX
                                     casadi.sumsqr(hat_par_REG_sym[10*i+2]/hat_par_REG_sym[10*i] - par_REG_link_i[2]/par_REG_link_i[0]) + # CY
                                     casadi.sumsqr(hat_par_REG_sym[10*i+3]/hat_par_REG_sym[10*i] - par_REG_link_i[3]/par_REG_link_i[0]))  # CZ
        f_inertia   +=  w_link[i] @ (casadi.sumsqr(hat_par_REG_sym[10*i+4] - par_REG_link_i[4]) +   # Ixx
                                     casadi.sumsqr(hat_par_REG_sym[10*i+5] - par_REG_link_i[5]) +   # Ixy
                                     casadi.sumsqr(hat_par_REG_sym[10*i+6] - par_REG_link_i[6]) +   # Ixz
                                     casadi.sumsqr(hat_par_REG_sym[10*i+7] - par_REG_link_i[7]) +   # Iyy
                                     casadi.sumsqr(hat_par_REG_sym[10*i+8] - par_REG_link_i[8]) +   # Iyz
                                     casadi.sumsqr(hat_par_REG_sym[10*i+9] - par_REG_link_i[9]))    # Izz

    # compute weight normalization at 1
    k=0
    for i in range(n):
        # par_REG_link_i = hat_par_REG_0[10*i:10*(i+1)]
        # W_normalization += W_link[i]*(W_mass/np.power(par_REG_link_i[0],2) + 
        #                               W_com/(np.power(par_REG_link_i[1]/par_REG_link_i[0],2)) +
        #                               W_com/(np.power(par_REG_link_i[2]/par_REG_link_i[0],2)) +
        #                               W_com/(np.power(par_REG_link_i[3]/par_REG_link_i[0],2)) + 
        #                               W_inertia/np.power(par_REG_link_i[4],2) + 
        #                               W_inertia/np.power(par_REG_link_i[5],2) +
        #                               W_inertia/np.power(par_REG_link_i[6],2) +
        #                               W_inertia/np.power(par_REG_link_i[7],2) +
        #                               W_inertia/np.power(par_REG_link_i[8],2) +
        #                               W_inertia/np.power(par_REG_link_i[9],2))
        # k += w_link[i]*(w_mass/(a*a) + 3*w_com*(a*a)/(b*b) + 6*w_inertia/(c*c))
        k += w_link[i]*(w_mass + 3*w_com + 6*w_inertia)
    #w_normalization = (1-w_loss)/k
    w_normalization = 1

    # Problem
    prob = {}
    prob['x'] = hat_par_REG_sym
    prob['f'] = w_loss* f_loss + (
                                    w_mass*f_mass +
                                    w_com*f_com +
                                    w_inertia*f_inertia
                                 )/ w_normalization
    prob['g'] = casadi.vertcat(*g)


    # convert cost terms into casadi functions
    f_loss_func = casadi.Function('f_loss_func',
                             [hat_par_REG_sym],
                             [w_loss*f_loss],
                             ['params'], ['cost'])
    f_mass_func = casadi.Function('f_mass_func',
                             [hat_par_REG_sym],
                             [w_mass/w_normalization*f_mass],
                             ['params'], ['cost'])
    f_com_func = casadi.Function('f_com_func',
                             [hat_par_REG_sym],
                             [w_com/w_normalization*f_com],
                             ['params'], ['cost'])
    f_inertia_func = casadi.Function('f_inertia_func',
                             [hat_par_REG_sym],
                             [w_inertia/w_normalization*f_inertia],
                             ['params'], ['cost'])


    # Options
    opts = {
        "ipopt": {
            "tol": 1e-4,
            "constr_viol_tol": 1e-6,
            "acceptable_tol": 1e-5,
            "dual_inf_tol": 1e-6,
            "compl_inf_tol": 1e-6,
            "max_iter": 50000,
        }
    }
    # Solver
    solver = casadi.nlpsol('sol', 'ipopt', prob, opts)

    sol = solver(x0=hat_par_REG_0, lbg=lb,ubg=ub)
    hat_par_REG_optim = sol['x'].full()

    # Compute residuals
    results = {}
    results['loss'] = f_loss_func(hat_par_REG_optim)
    results['mass'] = f_mass_func(hat_par_REG_optim)
    results['com'] = f_com_func(hat_par_REG_optim)
    results['inertia'] = f_inertia_func(hat_par_REG_optim)
    print(f"Loss: {results['loss']}, Mass: {results['mass']}, COM: {results['com']}, Inertia: {results['inertia']}")

    if export_file:
        # 1. Define and create the results directory
        if path is None:
            date_str = datetime.datetime.now().strftime("%d_%m_%Y")
            results_dir = f"results/data_{date_str}"
            if not os.path.exists(results_dir):
                os.makedirs(results_dir)
        else:
            results_dir = path
        # 2. Create identification_par.yaml
        config_path = os.path.join(results_dir, "identification_par.yaml")

        with open(config_path, "w") as f:
            # Add other parameters
            f.write(f"d3q: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"d4q: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"ddq: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"ddqr: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"dq: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"dqr: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"q: {[0 for _ in range(robot.numJoints)]}\n")
            f.write(f"w: {[0 for _ in range(6)]}\n")
            try:
                f.write(f"par_Ln2EE: {robot.get_par_Ln2EE().tolist()}\n")
            except AttributeError:
                pass
            try:
                f.write(f"par_world2L0: {robot.get_world2L0().tolist()}\n")
            except AttributeError:
                pass
            try:
                f.write(f"par_gravity: {robot.get_par_gravity().tolist()}\n")
            except AttributeError:
                pass

            # Add identified parameters
            f.write(f"par_DYN: {robot.get_par_DYN().tolist()}\n")
            f.write(f"par_REG: {robot.get_par_REG().tolist()}\n")
            f.write(f"par_Dl: {robot.get_par_Dl().tolist()}\n")
            f.write(f"par_REG_red: {robot.get_par_REG_red().tolist()}\n")
            try:
                # Check for actuator inertia or additional parameters
                f.write(f"par_Ia: {robot.get_par_Ia().tolist()}\n")
            except AttributeError:
                pass

        print(f"Estimated Dynamic Parameters written in '{config_path}'")

    if par_per_link == 10:
        return (hat_par_REG_optim, results)
    elif par_per_link == 11:
        hat_par_REG = hat_par_REG_optim[:n*10]
        hat_par_Ia = hat_par_REG_optim[n*10:n*11]
        return (hat_par_REG, hat_par_Ia, results)
    



def plot_identification(robot, traject, block = True):
    tau_M = []
    tau_C = []
    tau_G = []
    tau_dl = []
    tau_Ia = []
    delta_tau = []
    for i in range(traject.t.shape[0]):
        q = traject.q[i,:]
        dq = traject.qd[i,:]
        ddq = traject.qdd[i,:]

        robot.set_q(q)
        robot.set_dq(dq)
        robot.set_ddq(ddq)
        robot.set_dqr(dq)
        robot.set_ddqr(ddq)

        # Full inverse-dynamics regressor (per-joint rows)

        tau_est_M = robot.get_M()@ddq
        tau_est_C = robot.get_C()@dq
        tau_est_G = robot.get_G()
        tau_est_dl = robot.get_dl()
        try:
            tau_est_Ia = robot.get_Ia()
            tau_est = tau_est_M + tau_est_C + tau_est_G + tau_est_dl + tau_est_Ia
            tau_Ia.append(tau_est_Ia)
        except:
            tau_est = tau_est_M + tau_est_C + tau_est_G + tau_est_dl

        delta_tau.append(traject.tau[i,:] - tau_est.flatten())
        tau_M.append(tau_est_M)
        tau_C.append(tau_est_C)
        tau_G.append(tau_est_G)
        tau_dl.append(tau_est_dl)

    delta_tau = np.array(delta_tau)
    tau_M = np.array(tau_M)
    tau_C = np.array(tau_C)
    tau_G = np.array(tau_G)
    tau_dl = np.array(tau_dl)
    tau_Ia = np.array(tau_Ia)


    rmse = np.sqrt(np.mean(np.sum(delta_tau**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
    metrics = get_metrics(robot,traject)
    cond_num = metrics['conditioning number']
    # --- 1. Plot tau ---
    colors = {
        'measured': '#D62728',   # Strong Red
        'estimate': '#1F77B4',   # Strong Blue
        'mass':     '#2CA02C',   # Forest Green
        'coriolis': '#FF7F0E',   # Safety Orange
        'gravity':  '#9467BD',   # Royal Purple
        'friction': '#7F7F7F',   # Medium Gray
        'motor': '#7F0F7F'       # Medium Gray
    }
    n = robot.numJoints
    cols = int(np.sqrt(n)) 
    rows = (n + cols - 1) // cols  # Ceiling division to handle odd numbers
    JOINT_NAMES = traject.config['trajectory']['joints']

    fig = plt.figure(figsize=(12,8))
    plt.suptitle(f'Torque Estimation Results\nRMSE: {rmse:.4f} Nm | Conditioning Number (κ): {cond_num}', 
                fontsize=16, fontweight='bold')
    for i in range(n):
        plt.subplot(rows, cols,i+1)
        #plt.plot(t_log, tau_ori[:,i], label = 'tau')
        plt.plot(traject.t, traject.tau[:,i], 
                color=colors['measured'], linewidth=2, label='tau_filtered')

        plt.plot(traject.t, traject.tau[:,i] - delta_tau[:,i], 
                color=colors['estimate'], linewidth=1.5, label='hat_tau')

        # Individual components use dotted or dashed lines for clarity
        plt.plot(traject.t, tau_M[:,i], color=colors['mass'], 
                linestyle='--', alpha=0.8, label='hat_tau_M')

        plt.plot(traject.t, tau_C[:,i], color=colors['coriolis'], 
                linestyle=':', alpha=0.8, label='hat_tau_C')

        plt.plot(traject.t, tau_G[:,i], color=colors['gravity'], 
                linestyle='-.', alpha=0.8, label='hat_tau_G')

        plt.plot(traject.t, tau_dl[:,i], color=colors['friction'], 
                linestyle='dotted', alpha=0.7, label='hat_tau_dl')
        try:
            tau_est_Ia = robot.get_Ia()
            plt.plot(traject.t, tau_Ia[:,i], color=colors['motor'], 
                    linestyle='dotted', alpha=0.7, label='hat_tau_Ia')
        except:
            pass
        plt.xlabel('Time [s]')
        plt.ylabel('Torque [Nm]')
        plt.title(f'Torque {JOINT_NAMES[i]}')
        plt.grid(True)
        if i==0:
            plt.legend()
    plt.tight_layout()
    plt.show(block=block)

    return fig

def plot_LS_solution(hat_pi, metrics, pi_gt = None, block = True):
    # Create an array of indices for the x-axis
    x = np.arange(len(hat_pi))

    fig = plt.figure(figsize=(10, 6))

    # 1. Plot the ground truth (using green squares for clear contrast)
    if pi_gt is not None:
        plt.plot(x, pi_gt.flatten(), 'gs', markersize=6, label='Ground Truth')

    # 2. Plot hat_pi with its standard deviation as error bars
    # fmt='bo' uses blue circles for the points. ecolor sets the error bar color.
    plt.errorbar(x, hat_pi.flatten(), yerr=metrics['parameters standard deviation'].flatten(), fmt='bo', ecolor='red', 
                capsize=5, elinewidth=1.5, label='Estimated hat_pi')

    # Formatting
    plt.xlabel('Parameter Index')
    plt.ylabel('Parameter Value')
    plt.title('Parameter Estimation vs. Ground Truth')
    plt.xticks(x) # Ensures we get a tick for every parameter index
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show(block=block)
    return fig