
"""
Definition of function using CasADi framework
"""

import casadi
import numpy as np

def powerm(v, A, niter): #POWER ITERATION METHOD
    for i in range(niter):
        v = A @ v
        v = v/casadi.norm_2(v)
    l = v.T@A@v
    return v, l
def def_powerm(a, niter=None): #DEFLATION VERSION OF PIM FOR 3X3 MATRIX ---> CALCULATE ALL EIGENVALUES FROM GREATER TO SMALLER
     #AND THEIR EIGENVECTORS
     v0 = casadi.DM([1,1,1])

     if niter is None:
         niter = 40

     v1,l1 = powerm(v0, a, niter)

     v0 = casadi.DM([0,1,1])
     a2 = a - l1*(v1 @ v1.T)
     v2, l2 = powerm(v0, a2, niter)

     v0 = casadi.DM([0,0,1])
     a3 = a2 - l2 * (v2 @ v2.T)
     v3, l3 = powerm(v0, a3, niter)


     D = casadi.vertcat(l1,l2,l3)
     V = casadi.horzcat(v1,v2,v3)
     return V, D

sym_matrix = casadi.SX.sym('M', 3, 3)
eigenvalues = casadi.eig_symbolic(sym_matrix)

f_eigen = casadi.Function('eigen', [sym_matrix], [eigenvalues])

_, eigenvalues = def_powerm(sym_matrix)
f_powerm = casadi.Function('powerm', [sym_matrix], [eigenvalues])


#minimum eigenvalue
def minimum(eigen):
     min = 1e16
     for i in range(3):
         min = casadi.if_else(eigen[i] <= min, eigen[i], min)

     return min
def SN(eigen):
     p = 3
     l1 = eigen[0]**(-p)
     l2 = eigen[1]**(-p)
     l3 = eigen[2]**(-p)
     min = (l1 + l2 + l3)**(-1/p)


     return min
def det(A):
     if np.isscalar(A):
         return A
     elif A.shape[0]==1:
         return A[0,0]
     elif A.shape[0]==2:
         return (A[0,0]*A[1,1] - A[1,0]*A[0,1])
     elif A.shape[0]==3:
         return (A[0, 0]*(A[1, 1]*A[2, 2] - A[1, 2]*A[2, 1])
                 - A[0, 1]*(A[1, 0]*A[2, 2] - A[1, 2]*A[2, 0])
                 + A[0, 2]*(A[1, 0]*A[2, 1] - A[1, 1]*A[2, 0]))
     else:
         raise NotImplementedError("Supported dimensions: 1x1, 2x2, 3x3")
 # Skew-symmetric matrix
def skew(v):
     return np.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])
def casadi_skew(v):
     return casadi.vertcat(
         casadi.horzcat(0, -v[2], v[1]),
         casadi.horzcat(v[2], 0, -v[0]),
         casadi.horzcat(-v[1], v[0], 0)
     )

# Dynamic matrices
def dyn2reg(par_DYN, par_per_link = 10):
    par_REG = []
    for i in range(par_DYN.shape[0]//par_per_link):
        # Slice parameters for the current joint
        start_idx = par_per_link * i
        end_idx = par_per_link * (i + 1)
        p_dyn = par_DYN[start_idx:end_idx]
        # 1. Mass
        mass = p_dyn[0]

        # 2. First moment of mass (mass * Center of Mass)
        mCoM = mass * p_dyn[1:4]

        # 3. Inertia Tensor shift using Parallel Axis Theorem
        S = casadi_skew(mCoM)
        I_tmp = casadi.mtimes(S.T, S) / mass

        I_dyn = p_dyn[4:10]
        I_tmp_v = casadi.vertcat(
            I_tmp[0, 0], I_tmp[0, 1], I_tmp[0, 2],
                         I_tmp[1, 1], I_tmp[1, 2],
                                      I_tmp[2, 2]
        )

        I = I_dyn + I_tmp_v
        # Concatenate [mass, mCoM, I] for this joint
        par_REG.append(casadi.vertcat(mass, mCoM, I))
    # Stack all joints into a single vector
    par_REG = casadi.vertcat(*par_REG)
    return par_REG

def reg2dyn(par_REG, par_per_link = 10):
    par_DYN = []
    for i in range(par_REG.shape[0]//par_per_link):
        # Slice parameters for the current joint
        start_idx = par_per_link * i
        end_idx = par_per_link * (i + 1)
        p_reg = par_REG[start_idx:end_idx]

        # 1. Mass
        mass = p_reg[0]

        # 2. Center of Mass (First moment of mass / mass)
        CoM = p_reg[1:4] / mass

        # 3. Inertia Tensor shift (Reverse Parallel Axis Theorem)
        S = casadi_skew(CoM)
        I_tmp = mass * casadi.mtimes(S.T, S)

        I_reg = p_reg[4:10]
        I_tmp_v = casadi.vertcat(
            I_tmp[0, 0], I_tmp[0, 1], I_tmp[0, 2],
                         I_tmp[1, 1], I_tmp[1, 2],
                                      I_tmp[2, 2]
        )

        # Subtract the shift term to move inertia from origin back to CoM
        I = I_reg - I_tmp_v

        # Concatenate [mass, CoM, I] for this joint
        par_DYN.append(casadi.vertcat(mass, CoM, I))

    # Stack all joints into a single vector
    par_DYN = casadi.vertcat(*par_DYN)
    return par_DYN