# 导入所需的库
import numpy as np  # 用于数值计算
import scipy.linalg  # 用于线性代数运算
from cvxopt import solvers, matrix  # 用于求解二次规划问题
import matplotlib.pyplot as plt  # 用于绘图
from scipy.interpolate import interp1d

m = 1
# 定义系统状态转移矩阵 A
A = np.array([[1, 0.01], [0, 1]])
n = A.shape[0]  # 系统状态维度

# 定义输入矩阵 B
B = np.array([[0],[1/m]])
p = B.shape[1]  # 控制输入维度


# 定义权重矩阵
Q = np.array([[1, 0],[0, 1]])  # 状态权重矩阵
# F = np.array([[1, 0],[0, 1]])  # 终端状态权重矩阵

# 定义仿真步数和预测时域长度
k_steps = 50000 # 总仿真步数
N = 5  # 预测时域长度
R = np.zeros((N*n, 1))  # 初始化R为全零矩阵

# 随机生成几个控制点
num_control=4
control_points = [0.2, 0.3, 0.5, 0.5]  # 生成3个随机控制点
# 使用线性插值生成连续轨迹
# print()
f2 = interp1d(np.linspace(0, k_steps, num_control), control_points, kind='cubic')
path = f2(range(k_steps))

R[0::2,0] = path[:N]
# print(path)
w = 1
# 初始化状态和控制输入序列
X_k = np.zeros((n, k_steps))  # 存储系统状态轨迹
X_k[:,0] = [1, 0]  # 设置初始状态
U_k = np.zeros((p, k_steps))  # 存储控制输入序列

def cal_matrices(A,B,Q,w,N):
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
    n = A.shape[0] # 状态维数
    p = B.shape[1] # 输入维数
    
    # 构建预测矩阵 eye 仅有对角线
    M = np.zeros((N*n,n))  # 初始化状态预测矩阵 每个状态有两个分量
    C = np.zeros((N*n,N*p))  # 初始化输入影响矩阵
    W = np.diag([5., 4., 3., 2., 1.]) * w
    tmp = np.eye(n)
    
    # 构建预测方程矩阵
    for i in range(N):
        rows = i * n 
        B_tmp = np.dot(tmp, B)
        for j in range(N):
            if i >= j:
                cols = j * p
                C[rows:rows+n,cols:cols+p] = B_tmp
        tmp = np.dot(A, tmp)
        M[rows:rows+n,:] = tmp
    print(C)
    # print(M)
    # 构建权重矩阵
    Q_bar = np.kron(np.eye(N), Q)  # 状态权重扩展
    # Q_bar = scipy.linalg.block_diag(Q_bar_be, F)  # 包含终端权重的完整权重矩阵
    # print(Q_bar.shape)

    # 二次规划的两个矩阵
    H = np.matmul(np.matmul(C.transpose(),Q_bar),C) + W
    E = 2 * np.matmul(Q_bar, C) 
    # print(E)
    return H, E ,M

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
    u_k = U_thk[0]  # 提取当前时刻的控制输入
    # print(U_thk)
    return u_k

# 计算MPC所需矩阵
H, E, M = cal_matrices(A,B,Q,w,N)
H = matrix(H)  # 转换为CVXOPT矩阵格式

# 主循环 - 系统仿真
for k in range(1,k_steps-5):
    # 获取当前状态和控制输入
    x_kshort = X_k[:, k-1].reshape(-1, 1)
    u_kshort = U_k[:, k-1].reshape(-1, 1)
    # print(path[k+N-1:k+N+4])
    R[0::2,0] = path[k:k+5]
    # 计算优化问题的参数
    T = np.dot((np.matmul(M, x_kshort)-R).T, E).T
    T = matrix(T)

    pred = Prediction(H,T)
    U_k[:,k] = pred
    pred = np.expand_dims(pred, axis=1)
    # 更新系统状态
    X_knew = np.matmul(A,x_kshort) + np.matmul(B,pred)
 
    # print(f"current state {x_kshort} control {u_kshort} prediction {pred} next_state {X_knew}")
    # print(X_knew)
    # 存储新状态
    X_k[:,k] = X_knew.reshape(2)

# 打印状态轨迹
# print(X_k)

# 可视化位置和速度
plt.figure(figsize=(12, 6))

# 位置子图
plt.subplot(2, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(X_k[0, 1:-N], label='Position', color='b')
plt.plot(path[1:-N], color='r', linestyle='--', label='y=0')  # Horizontal line at y=0
plt.title('Position over Time')
plt.xlabel('Time Steps')
plt.ylabel('Position')
plt.legend()
plt.grid()

# 速度子图
plt.subplot(2, 1, 2)  # 2 rows, 1 column, 2nd subplot
plt.plot(X_k[1, 1:-N], label='Velocity', color='g')
plt.title('Velocity over Time')
plt.xlabel('Time Steps')
plt.ylabel('Velocity')
plt.legend()
plt.grid()

plt.tight_layout()  # Adjust layout to prevent overlap
plt.show()