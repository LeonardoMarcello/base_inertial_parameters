"""
utilities function for the evaluation of identification process 
against a ground truth robot model.
"""

import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from utils.identification_utils import *

def plot_eval_identification(robot, metrics, traject, robot_gt = None, block = True):
    """ Plot measurement and reconstructed effort """
    delta_tau = []
    delta_tau_GT = []

    pi_red = robot.get_par_REG_red()
    pi_dl = robot.get_par_Dl()
    pi_red_CAD = robot_gt.get_par_REG_red()
    pi_dl_CAD = robot_gt.get_par_Dl()


    for i in range(traject.t.shape[0]):
        q = traject.q[i,:]
        dq = traject.qd[i,:]
        ddq = traject.qdd[i,:]

        robot.set_q(q)
        robot.set_dq(dq)
        robot.set_ddq(ddq)
        robot.set_dqr(dq)
        robot.set_ddqr(ddq)

        Y_red = robot.get_Yr_red()
        Y_dl = robot.get_reg_dl()

        tau_est = (Y_red@pi_red + Y_dl@pi_dl).flatten()
        delta_tau.append(traject.tau[i,:] - tau_est)
        delta_tau_GT.append(traject.tau[i,:] - (Y_red @ pi_red_CAD + Y_dl @ pi_dl_CAD).flatten())
    delta_tau = np.array(delta_tau)
    delta_tau_GT = np.array(delta_tau_GT)


    rmse = np.sqrt(np.mean(np.sum(delta_tau**2, axis=1))) # RMSE = Sqrt{ 1/M Sum{||e||_2}  }
    print(f"RMSE: {rmse} [Nm]" )

    cond_num = metrics['conditioning number']
    # --- 1. Plot tau ---
    colors = {
        'measured': '#D62728',   # Strong Red
        'estimate': '#1F77B4',   # Strong Blue
        'ground-truth':     '#2CA02C',   # Forest Green
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

        plt.plot(traject.t, traject.tau[:,i] - delta_tau_GT[:,i], 
                color=colors['ground-truth'], linewidth=1.5, label='tau_reconstructed')

        plt.xlabel('Time [s]')
        plt.ylabel('Torque [Nm]')
        plt.title(f'Torque {JOINT_NAMES[i]}')
        plt.grid(True)
        if i==0:
            plt.legend()
    plt.tight_layout()
    plt.show(block=block)

    return fig




def plot_eval_LS_solution(hat_pi, metrics, robot_gt = None, block = True):
    """ Plot Base parameters estim as boxplot """
    # Create an array of indices for the x-axis
    x = np.arange(len(hat_pi))

    fig = plt.figure(figsize=(10, 6))

    # 1. Plot the ground truth (using green squares for clear contrast)
    if robot_gt is not None:
        plt.plot(x, np.hstack([robot_gt.get_par_REG_red(),robot_gt.get_par_Dl()]).flatten(), 'gs', markersize=6, label='Ground Truth')

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
    plt.show(block = block)
    return fig


def plot_table(robot, robot_ground_truth, format = 'plain'):
    """ Print Table of Dynamic parameters """
    np.set_printoptions(precision=4, suppress=True, linewidth=200)
    if hasattr(robot, "get_par_Ia"):
        par_per_link = 11
    else:
        par_per_link = 10
    n = robot.numJoints
    headers = ["m", "cx", "cy", "cz", "ixx", "ixy", "ixz", "iyy", "iyz", "izz",'Ia']
    headers = headers[:par_per_link]


    if format.lower() == 'latex':
        # 1) Optimized params
        if par_per_link == 11:
            params = np.hstack([robot.get_par_DYN().reshape(n,10), robot.get_par_Ia().reshape(n,1)]).reshape(n, 11)
        else:
            params = robot.get_par_DYN().reshape((n, 10))
        title = "Optimized parameters"
        col_format = "c|" + "r" * len(headers)
        print(r"\begin{table}[htpb]")
        print(r"\centering")
        print(rf"\caption{{{title}}}")
        # Requires \usepackage{graphicx} in your LaTeX preamble
        print(r"\resizebox{\textwidth}{!}{%")
        print(rf"\begin{{tabular}}{{{col_format}}}")
        print(r"\hline")

        # Print Headers
        header_str = "Link & " + " & ".join([f"${h}$" for h in headers]) + r" \\ \hline"
        print(header_str)

        # Print Rows
        for i, row in enumerate(params):
            row_strs = []
            for val in row:
                # Convert to LaTeX scientific notation (e.g. 1.23e-04 -> $1.23 \times 10^{-4}$)
                val_e = f"{val:.6e}"
                base, exp = val_e.split('e')
                exp_int = int(exp) # Removes leading zeros from exponent
                row_strs.append(f"${base} \\times 10^{{{exp_int}}}$")
            print(f"{i} & " + " & ".join(row_strs) + r" \\")
        print(r"\hline")
        print(r"\end{tabular}%")
        print(r"}")
        print(r"\end{table}")
        print() # Add spacing between tables

        if robot_ground_truth is not None:
            # compute error model
            e_DYN = np.abs(robot_ground_truth.get_par_DYN() - robot.get_par_DYN())
            e_model = e_DYN.reshape((n, 10))
            if par_per_link == 11:
                e_Ia = np.abs(robot_ground_truth.get_par_Ia() - robot.get_par_Ia())
                e_model = np.hstack([e_DYN.reshape(n,10), e_Ia.reshape(n,1)]).reshape(n, 11)

            # 2) Robot Ground-Truth ------------------------
            title = "Robot model ground-truth (DYNAMIC)"
            col_format = "c|" + "r" * len(headers)
            print(r"\begin{table}[htpb]")
            print(r"\centering")
            print(rf"\caption{{{title}}}")
            # Requires \usepackage{graphicx} in your LaTeX preamble
            print(r"\resizebox{\textwidth}{!}{%")
            print(rf"\begin{{tabular}}{{{col_format}}}")
            print(r"\hline")

            # Print Headers
            header_str = "Link & " + " & ".join([f"${h}$" for h in headers]) + r" \\ \hline"
            print(header_str)

            # Print Rows
            for i, row in enumerate(params):
                row_strs = []
                for val in row:
                    # Convert to LaTeX scientific notation (e.g. 1.23e-04 -> $1.23 \times 10^{-4}$)
                    val_e = f"{val:.6e}"
                    base, exp = val_e.split('e')
                    exp_int = int(exp) # Removes leading zeros from exponent
                    row_strs.append(f"${base} \\times 10^{{{exp_int}}}$")

                print(f"{i} & " + " & ".join(row_strs) + r" \\")

            print(r"\hline")
            print(r"\end{tabular}%")
            print(r"}")
            print(r"\end{table}")
            print() # Add spacing between tables

            # 3) Error ------------------------
            title = "Variation from ground-truth (absolute error)"
            col_format = "c|" + "r" * len(headers)
            print(r"\begin{table}[htpb]")
            print(r"\centering")
            print(rf"\caption{{{title}}}")
            # Requires \usepackage{graphicx} in your LaTeX preamble
            print(r"\resizebox{\textwidth}{!}{%")
            print(rf"\begin{{tabular}}{{{col_format}}}")
            print(r"\hline")

            # Print Headers
            header_str = "Link & " + " & ".join([f"${h}$" for h in headers]) + r" \\ \hline"
            print(header_str)

            # Print Rows
            for i, row in enumerate(params):
                row_strs = []
                for val in row:
                    # Convert to LaTeX scientific notation (e.g. 1.23e-04 -> $1.23 \times 10^{-4}$)
                    val_e = f"{val:.6e}"
                    base, exp = val_e.split('e')
                    exp_int = int(exp) # Removes leading zeros from exponent
                    row_strs.append(f"${base} \\times 10^{{{exp_int}}}$")

                print(f"{i} & " + " & ".join(row_strs) + r" \\")

            print(r"\hline")
            print(r"\end{tabular}%")
            print(r"}")
            print(r"\end{table}")
            print() # Add spacing between tables
    else:
        # 1) Optimized params
        print(f"Optimized parameters")
        if par_per_link == 11:
            params = np.hstack([robot.get_par_DYN().reshape(n,10), robot.get_par_Ia().reshape(n,1)]).reshape(n, 11)
        else:
            params = robot.get_par_DYN().reshape((n, 10))
        # Print header
        header_str = f"{'Link':<6}" + "".join([f"{h:>14}" for h in headers])
        print(header_str)
        print("-" * len(header_str))
        # Print rows
        for i, row in enumerate(params):
            row_str = f"{i:<6}" + "".join([f"{val:>14.6e}" for val in row])
            print(row_str)

        if robot_ground_truth is not None:
            # compute error model
            e_DYN = np.abs(robot_ground_truth.get_par_DYN() - robot.get_par_DYN())
            e_model = e_DYN.reshape((n, 10))
            if par_per_link == 11:
                e_Ia = np.abs(robot_ground_truth.get_par_Ia() - robot.get_par_Ia())
                e_model = np.hstack([e_DYN.reshape(n,10), e_Ia.reshape(n,1)]).reshape(n, 11)

            # 2) Robot Ground-Truth ------------------------
            print(f"Robot model ground-truth (DYNAMIC)")
            if par_per_link == 11:
                params = np.hstack([robot.get_par_DYN().reshape(n,10), robot.get_par_Ia().reshape(n,1)]).reshape(n, 11)
            else:
                params = robot_ground_truth.get_par_DYN().reshape((n, 10))
            # Print header
            header_str = f"{'Link':<6}" + "".join([f"{h:>14}" for h in headers])
            print(header_str)
            print("-" * len(header_str))
            # Print rows
            for i, row in enumerate(params):
                row_str = f"{i:<6}" + "".join([f"{val:>14.6e}" for val in row])
                print(row_str)

            # 3) Variation from  Ground-Truth ------------------------
            print(f"Variation from ground-truth (absolute error)")
            params = e_model
            # Print header
            header_str = f"{'Link':<6}" + "".join([f"{h:>14}" for h in headers])
            print(header_str)
            print("-" * len(header_str))
            # Print rows
            for i, row in enumerate(params):
                row_str = f"{i:<6}" + "".join([f"{val:>14.6e}" for val in row])
                print(row_str)


def plot_link_solution(robot, robot_gt, n, title = None, block = True):
    # Ground truth
    m_gt = robot_gt.get_par_DYN()[n*10 + 0]
    com_gt = np.array([robot_gt.get_par_DYN()[n*10 + 1],
                       robot_gt.get_par_DYN()[n*10 + 2],
                       robot_gt.get_par_DYN()[n*10 + 3]])
    Ib_gt = np.array([ robot_gt.get_par_DYN()[n*10 + 4],
                       robot_gt.get_par_DYN()[n*10 + 5],
                       robot_gt.get_par_DYN()[n*10 + 6],
                       robot_gt.get_par_DYN()[n*10 + 7],
                       robot_gt.get_par_DYN()[n*10 + 8],
                       robot_gt.get_par_DYN()[n*10 + 9]])
    # Estimation
    m_estim  = robot.get_par_DYN()[n*10 + 0]
    com_estim  = np.array([robot.get_par_DYN()[n*10 + 1],
                           robot.get_par_DYN()[n*10 + 2],
                           robot.get_par_DYN()[n*10 + 3]])
    Ib_estim  = np.array([ robot.get_par_DYN()[n*10 + 4],
                           robot.get_par_DYN()[n*10 + 5],
                           robot.get_par_DYN()[n*10 + 6],
                           robot.get_par_DYN()[n*10 + 7],
                           robot.get_par_DYN()[n*10 + 8],
                           robot.get_par_DYN()[n*10 + 9]])

    # 3. Prepare data for plotting
    com_labels = ['x', 'y', 'z']
    inertia_labels = ['Ixx', 'Iyy', 'Izz', 'Ixy', 'Ixz', 'Iyz']

    urdf_inertia = [Ib_gt[0], Ib_gt[3], Ib_gt[5], 
                    Ib_gt[1], Ib_gt[2], Ib_gt[4]]
    params_inertia = [Ib_estim[0], Ib_estim[3], Ib_estim[5], 
                    Ib_estim[1], Ib_estim[2], Ib_estim[4]]

    # 4. Create Subplots
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    if title is not None:
        fig.suptitle(title)
    else:
        fig.suptitle(f"Estimation of link {n}")

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
    plt.show(block = block)
    return fig