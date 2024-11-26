# mpc_module.py

import numpy as np
import scipy.linalg
from cvxopt import solvers, matrix
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def compute_ABO(theta_r, v_r, t):
    """
    Compute the matrices for the discrete-time linearized model.
    
    Parameters:
    theta_r: float - Reference orientation
    v_r: float - Reference velocity
    t: float - Time step
    
    Returns:
    A_hat, B_hat, O_hat: np.ndarray - Discrete-time linearized matrices
    """
    A_hat = np.array([
        [1, 0, -t * v_r * np.sin(theta_r)],
        [0, 1, t * v_r * np.cos(theta_r)],
        [0, 0, 1]
    ])
    
    B_hat = np.array([
        [t * np.cos(theta_r), 0],
        [t * np.sin(theta_r), 0],
        [0, t]
    ])
    
    O_hat = np.array([
        [theta_r * t * v_r * np.sin(theta_r)],
        [-theta_r * t * v_r * np.cos(theta_r)],
        [0]
    ])

    return A_hat, B_hat, O_hat

def cal_matrices(A, B, Q, N, n, p):
    """
    Calculate the matrices required for MPC
    Parameters:
    A: State transition matrix
    B: Input matrix
    Q: State weight matrix
    N: Prediction horizon length
    n: State dimension
    p: Control input dimension
    """
    M = np.zeros((N*n, n))
    C = np.zeros((N*n, N*p))
    D = np.zeros((N*n, N*n))
    W = np.diag(np.linspace(2, 0, N))
    # W = np.eye(N)
    weight = np.eye(p)
    W = np.kron(W, weight)
    tmp = np.eye(n)
    
    for i in range(N):
        rows = i * n
        B_tmp = np.dot(tmp, B)
        for j in range(N-i):
            cols_C = j * p
            cols_D = j * n
            C[rows+j*n:rows+j*n+n, cols_C:cols_C+p] = B_tmp
            D[rows+j*n:rows+j*n+n, cols_D:cols_D+n] = tmp
        tmp = np.dot(tmp, A)
        M[rows:rows+n, :] = tmp

    Q_bar = Q
    H = np.matmul(np.matmul(C.transpose(), Q_bar), C) + W
    QC = 2 * np.matmul(Q_bar, C)
    return H, QC, M, D , C

def Prediction(H, T, p):
    """
    Solve the quadratic programming problem to obtain the optimal control input
    Parameters:
    H: Quadratic term coefficient matrix
    T: Linear term coefficient vector
    p: Control input dimension
    """
    sol = solvers.qp(H, T)
    U_thk = np.array(sol["x"])
    u_k = U_thk[:p, 0]
    return u_k, U_thk

def simulate_mpc(k_steps, N, t, num_control, control_points, initial_state, num_samples=20):
    """
    Simulate the MPC controller.
    
    Parameters:
    k_steps: int - Total simulation steps
    N: int - Prediction horizon length
    t: float - Time step
    num_control: int - Number of control points
    control_points: list - Control points for the reference path
    initial_state: list - Initial state of the system
    
    Returns:
    X_k: np.ndarray - System state trajectory
    U_k: np.ndarray - Control input sequence
    """
    sample_interval = k_steps // num_samples
    pred_list=[]
    v_r = 0.0
    theta_r = 0.0
    A_hat, B_hat, O_hat = compute_ABO(theta_r, v_r, t)
    n = A_hat.shape[0]
    p = B_hat.shape[1]
    Q0 = np.eye(n)
    Q0[-1,-1] = 1
    Q = np.kron(np.eye(N), Q0)
    R = np.zeros((N * n, 1))
    
    X = np.linspace(0, 1, k_steps)
    f2 = interp1d(np.linspace(0, k_steps, num_control), control_points, kind='cubic')
    path = f2(range(k_steps))
    path_diff = np.diff(path)
    path_tan = path_diff * k_steps
    angles = np.arctan(path_tan)
    angles = np.append(angles, angles[-1])
    
    R[0::n, 0] = X[:N]
    R[1::n, 0] = path[:N]
    R[2::n, 0] = angles[:N]
    
    X_k = np.zeros((n, k_steps))
    X_k[:, 0] = initial_state
    U_k = np.zeros((p, k_steps))
    
    for k in range(1, k_steps - N):

        x_kshort = X_k[:, k-1].reshape(-1, 1)
        u_kshort = U_k[:, k-1].reshape(-1, 1)
        R[0::n, 0] = X[k:k+N]
        R[1::n, 0] = path[k:k+N]
        R[2::n, 0] = angles[k:k+N]
        
        A_hat, B_hat, O_hat = compute_ABO(x_kshort[2, 0], u_kshort[0, 0], t)
        H, QC, M, D, C = cal_matrices(A_hat, B_hat, Q, N, n, p)
        O = np.kron(O_hat, np.ones((N, 1)))
        H = matrix(H)
        T = np.dot((np.matmul(M, x_kshort) + np.matmul(D, O) - R).T, QC).T
        T = matrix(T)
        
        pred, U_thk = Prediction(H, T, p)
        if k%sample_interval == 0 or k == 1 or k == k_steps - N-1:
            X_pred = np.matmul(M, x_kshort) + np.matmul(C, U_thk) + np.matmul(D, O)
            pred_list.append(X_pred.reshape(N, n))

        U_k[:, k] = pred.reshape(p)
        pred = np.expand_dims(pred, axis=1)
        
        X_knew = np.matmul(A_hat, x_kshort) + np.matmul(B_hat, pred) + O_hat
        X_k[:, k] = X_knew.reshape(n)
    
    return X_k, U_k, X, path, pred_list

def plot_trajectory(X_k, X, path, N):
    """
    Plot the trajectory of the system.
    
    Parameters:
    X_k: np.ndarray - System state trajectory
    X: np.ndarray - Reference path x-coordinates
    path: np.ndarray - Reference path y-coordinates
    N: int - Prediction horizon length
    """
    plt.plot(X[:-N], path[:-N], label='Reference', color='r')
    plt.plot(X_k[0, :-N], X_k[1, :-N], label='Position', color='b')
    plt.title('Position over Time')
    plt.xlabel('Time Steps')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()
    plt.tight_layout()