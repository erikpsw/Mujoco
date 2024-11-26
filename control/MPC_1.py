# 导入所需的库
import numpy as np  # 用于数值计算
import scipy.linalg  # 用于线性代数运算
from cvxopt import solvers, matrix  # 用于求解二次规划问题
import matplotlib.pyplot as plt  # 用于绘图
from scipy.interpolate import interp1d

def compute_ABO(theta_r, v_r, t):
    """
    Compute the matrices for the discrete-time linearized model.
    
    Parameters:
    x_r, y_r, theta_r: float - Reference state variables
    v_r: float - Reference velocity
    T: float - Time step
    
    Returns:
    A_hat, B_hat, O_hat: np.ndarray - Discrete-time linearized matrices
    """
    # Compute the A_hat matrix
    A_hat = np.array([
        [1, 0, -t * v_r * np.sin(theta_r)],
        [0, 1, t * v_r * np.cos(theta_r)],
        [0, 0, 1]
    ])
    
    # Compute the B_hat matrix
    B_hat = np.array([
        [t * np.cos(theta_r), 0],
        [t * np.sin(theta_r), 0],
        [0, t]
    ])
    
    # Compute the O_hat matrix
    O_hat = np.array([
        [theta_r * t * v_r * np.sin(theta_r)],
        [- theta_r * t *  v_r * np.cos(theta_r)],
        [0]
    ])

    return A_hat, B_hat, O_hat


def cal_matrices(A,B,Q,N):
    """
    计算MPC所需的矩阵
    参数:
    A: 状态转移矩阵
    B: 输入矩阵
    Q: 状态权重矩阵
    R: 输入权重矩阵
    F: 终端权重矩阵
    N: 预测时域长度
    """
    
    # 构建预测矩阵 eye 仅有对角线
    M = np.zeros((N*n,n))  # 初始化状态预测矩阵 每个状态有两个分量
    C = np.zeros((N*n,N*p))  # 初始化输入影响矩阵
    D = np.zeros((N*n,N*n))  # 初始化常数矩阵
    W = np.diag(np.linspace(2,1,N)) 
    weight = np.eye(p)
    # weight[1,1] = 10
    W = np.kron(W, weight)
    # print(W)
    tmp = np.eye(n)
    
    # 构建预测方程矩阵
    for i in range(N):
        rows = i * n 
        B_tmp = np.dot(tmp, B)
        for j in range(N-i):
            cols_C = j * p
            cols_D = j * n
            C[rows+j*n:rows+j*n+n,cols_C:cols_C+p] = B_tmp
            D[rows+j*n:rows+j*n+n,cols_D:cols_D+n] = tmp
        tmp = np.dot(tmp,A)
        M[rows:rows+n,:] = tmp

    # print(M)
    # 构建权重矩阵
    Q_bar = Q  # 状态权重扩展
    # Q_bar = scipy.linalg.block_diag(Q_bar_be, F)  # 包含终端权重的完整权重矩阵
    # print(Q_bar.shape)

    # 二次规划的两个矩阵
    H = np.matmul(np.matmul(C.transpose(),Q_bar),C) + W
    QC = 2 * np.matmul(Q_bar, C) 
    # print(E)
    return H, QC, M, D

def Prediction(M,T):
    """
    求解二次规划问题获取最优控制输入
    参数:
    M: 二次项系数矩阵
    T: 线性项系数向量
    """
    sol = solvers.qp(M,T)  # 求解二次规划问题
    U_thk = np.array(sol["x"])  # 获取最优解
    # print(U_thk)
    u_k = U_thk[:p,0]  # 提取当前时刻的控制输入
    # print(u_k)
    return u_k

t = 0.05  # 时间步长
v_r = 0.0
theta_r = 0.0

# Compute the matrix
A_hat, B_hat, O_hat = compute_ABO(theta_r, v_r, t)

n = A_hat.shape[0]  # 系统状态维度
p = B_hat.shape[1]  # 控制输入维度

# 定义仿真步数和预测时域长度
k_steps = 2000 # 总仿真步数
N = 50 # 预测时域长度
# 定义权重矩阵
Q = np.eye(n*N)  # 状态权重矩阵
# F = np.array([[1, 0],[0, 1]])  # 终端状态权重矩阵
R = np.zeros((N*n, 1))  # 初始化R为全零矩阵

# R[2::3,0] = np.ones(N)*0 # 设置x方向的参考轨迹
# print(R)
# 随机生成几个控制点
num_control=4
control_points = [0.2, 0.3, 0.5, 0.9]  # 生成3个随机控制点
# 使用线性插值生成连续轨迹

X = np.linspace(0, 1, k_steps)
f2 = interp1d(np.linspace(0, k_steps, num_control), control_points, kind='cubic')
path = f2(range(k_steps))
# Calculate the derivative of the path
path_diff = np.diff(path)
path_tan = path_diff*k_steps
# Calculate the angles of the tangents
angles = np.arctan(path_tan)

# Append the last angle to match the length of the path
angles = np.append(angles, angles[-1])

# # 单个轨迹点
# R[0::n,0] = np.ones(N)  # 设置x方向的参考轨迹
# R[1::n,0] = np.ones(N)  # 设置x方向的参考轨迹
# R[2::n,0] = np.ones(N)*np.pi/2 # 设置x方向的参考轨迹

# print(path)
R[0::n,0] = X[:N]
R[1::n,0] = path[:N]
R[2::n,0] = angles[:N]
# print(R)

# 初始化状态和控制输入序列
X_k = np.zeros((n, k_steps))  # 存储系统状态轨迹
X_k[:,0] = [-0.1, 0.1, np.pi/2]  # 设置初始状态
U_k = np.zeros((p, k_steps))  # 存储控制输入序列

# 主循环 - 系统仿真
for k in range(1,k_steps-N):
    # 获取当前状态和控制输入
    x_kshort = X_k[:, k-1].reshape(-1, 1)
    u_kshort = U_k[:, k-1].reshape(-1, 1)
    # print(u_kshort)
    # 计算优化问题的参数
    # print(path)
    R[0::n,0] = X[k:k+N]
    R[1::n,0] = path[k:k+N]
    R[2::n,0] = angles[k:k+N]
    # print 路径
    # print(R)      
    A_hat, B_hat, O_hat = compute_ABO(x_kshort[2,0], u_kshort[0,0], t)
    H, QC, M, D = cal_matrices(A_hat, B_hat, Q, N)
    O = np.kron(O_hat, np.ones((N,1)))
    H = matrix(H)  # 转换为CVXOPT矩阵格式
    # 打印各个矩阵的形状
    # print(f"M {M.shape} X_kshort {x_kshort.shape} D {D.shape} O {O.shape} QC {QC.shape}")
    T = np.dot((np.matmul(M, x_kshort) + np.matmul(D, O)-R).T, QC).T
    T = matrix(T)
    
    pred = Prediction(H,T)
    U_k[:,k] = pred.reshape(p)
    pred = np.expand_dims(pred, axis=1)
    
    # 打印形状
    # print(f" A_hat {A_hat.shape} x_kshort {x_kshort.shape} B_hat {B_hat.shape} pred {pred.shape} O_hat {O_hat.shape}")
    # 更新系统状态
    X_knew = np.matmul(A_hat,x_kshort) + np.matmul(B_hat,pred) + O_hat
 
    # print(f"current state {x_kshort} control {u_kshort} prediction {pred} next_state {X_knew}")
    # print(X_knew)
    # 存储新状态
    X_k[:,k] = X_knew.reshape(n)

# 打印状态轨迹
# print(angles)

# 可视化位置和速度
plt.figure(figsize=(12, 6))

plt.plot(X[:-N], path[:-N], label='Reference', color='r')  # 位置参考轨迹
# 位置子图
plt.plot(X_k[0, :-N],X_k[1, :-N], label='Position', color='b')
# plt.plot(angles*180/np.pi, label='Reference', color='r')  # 角度参考轨迹

plt.title('Position over Time')
plt.xlabel('Time Steps')
plt.ylabel('Position')
plt.legend()
plt.grid()


plt.tight_layout()  # Adjust layout to prevent overlap
plt.show()