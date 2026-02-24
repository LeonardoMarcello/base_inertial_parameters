
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