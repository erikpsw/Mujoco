import numpy as np
import json  # Neu hinzugefügt
from dm_control import mujoco
import mujoco_viewer
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from cvxopt import matrix, solvers
import MPC_solver
import dynamic_model_linear
import math

# Implementierung der Funktion compute_ABO
def compute_ABO(theta_r, v_r, t):
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

# Implementierung der Funktion cal_matrices
def cal_matrices(A_hat, B_hat, Q, R_input, N, n, p):
    M = np.zeros((N * n, n))
    C = np.zeros((N * n, N * p))

    for i in range(N):
        A_i = np.linalg.matrix_power(A_hat, i + 1)
        M[i * n:(i + 1) * n, :] = A_i
        for j in range(i + 1):
            A_j = np.linalg.matrix_power(A_hat, i - j)
            idx_start = j * p
            idx_end = (j + 1) * p
            C[i * n:(i + 1) * n, idx_start:idx_end] += np.dot(A_j, B_hat)

    # H = 2 * (C^T * Q * C + R_input)
    H = 2 * (np.dot(C.T, np.dot(Q, C)) + R_input)
    return H, M, C

# Implementierung der Funktion Prediction
def Prediction(H, T, p):
    H_cvx = matrix(H)
    T_cvx = matrix(T)

    # Deaktivieren der Ausgaben des Solvers
    solvers.options['show_progress'] = False

    # Definieren von G und h als leere Matrizen
    n_vars = H_cvx.size[0]
    G_cvx = matrix(0.0, (0, n_vars))
    h_cvx = matrix(0.0, (0, 1))

    # Lösen des QP-Problems
    sol = solvers.qp(H_cvx, T_cvx, G_cvx, h_cvx)
    U_thk = np.array(sol['x']).flatten()
    pred = U_thk[:p]
    return pred, U_thk

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


# Hauptskript
if __name__ == "__main__":
    # Modellpfad anpassen
    model_path = '../model/mini_mec_six_arm_cylinder.xml'  # HIER ANPASSEN
    model = mujoco.MjModel.from_xml_path(model_path)
    physics = mujoco.Physics.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建渲染器
    viewer = mujoco_viewer.MujocoViewer(model, data)

    # 定义关节控制函数
    def control_joints(joint_ids, positions):
        for joint_id, position in zip(joint_ids, positions):
            data.ctrl[joint_id] = position  # 使用 actuator 来控制关节位置

    def control_vel(joint_ids, velocities):
        for joint_id, velocity in zip(joint_ids, velocities):
            data.ctrl[joint_id] = velocity  # 使用 actuator 来控制关节速度



    # 设置MPC参数
    k_steps = 5000  # Sie können diesen Wert erhöhen, um den gesamten Pfad abzudecken
    N = 4
    simulation_step = 10
    t = 0.002 * simulation_step
    t = 0.4
    r = 0.028
    N_target = 1000

    initial_state = np.array([0, 0, 0])
    control_points = [0, 0.4, 0.2, 0.6]
        # Anzahl der Kontrollpunkte
    num_control = len(control_points)

    pred_list = []
    v_r = 0.0
    theta_r = 0.0
    A_hat, B_hat, O_hat = MPC_solver.compute_ABO(theta_r, v_r, t)
    n = A_hat.shape[0]
    p = B_hat.shape[1]

    mojoco_origin = np.array(data.geom_xpos[2:6]).mean(axis=0)[:n]
    # print(mojoco_origin)
    Q0 = np.eye(n)
    Q0[-1, -1] = 2
    Q = np.kron(np.eye(N), Q0)
    R = np.zeros((N * n, 1))
    X = np.linspace(0, 2, k_steps)
    f2 = interp1d(np.linspace(0, k_steps, num_control), control_points, kind='cubic')
    path = f2(range(k_steps))
    path_diff = np.diff(path)
    path_tan = path_diff * k_steps
    angles = np.arctan(path_tan)
    angles = np.append(angles, angles[-1])
    R[0::n, 0] = X[:N]
    R[1::n, 0] = path[:N]
    R[2::n, 0] = angles[:N]
    X_k = np.zeros((n, k_steps + N_target))
    X_k[:, 0] = initial_state
    U_k = np.zeros((p, k_steps + N_target))

    # MPC_solver.plot_trajectory(X_k, X, path, N)
    # plt.show()
    vhc = dynamic_model_linear.DifferentialDriveRobot()

    # # 绘制图像
    # plt.figure()
    # plt.plot(V_L/r, label='Left Wheel Speed (v_L)')
    # plt.plot(V_R/r, label='Right Wheel Speed (v_R)')
    # plt.xlabel('Time Step')
    # plt.ylabel('Speed')
    # plt.title('Left and Right Wheel Speeds')
    # plt.legend()
    # plt.show()

    # 初始化轨迹点列表
    trajectory = []
    max_traj_length = 2000
    wheel_pos_list = []
    model_trajectory = []
    W = vhc.W

    # Hauptsimulationsschleife
    k = 1  # Schrittzähler für MPC
    while viewer.is_alive and k < k_steps - N + N_target:
        # print(physics.timestep())
        # 控制关节角度（如果有必要）
        joint_angles = [0, 0, 0, 0, 0, 0]
        control_joints([0, 1, 2, 3, 4, 5], joint_angles)
        x_kshort = X_k[:, k-1].reshape(-1, 1)
        u_kshort = U_k[:, k-1].reshape(-1, 1)
        if k < k_steps - N:
            R[0::n, 0] = X[k:k+N]
            R[1::n, 0] = path[k:k+N]
            R[2::n, 0] = angles[k:k+N]
        else:
            R[0::n, 0] = X[-1]
            R[1::n, 0] = path[-1]
            R[2::n, 0] = angles[-1]
        A_hat, B_hat, O_hat = MPC_solver.compute_ABO(x_kshort[2, 0], u_kshort[0, 0], t)
        H, QC, M, D, C = MPC_solver.cal_matrices(A_hat, B_hat, Q, N, n, p)
        O = np.kron(O_hat, np.ones((N, 1)))
        H = matrix(H)
        T = np.dot((np.matmul(M, x_kshort) + np.matmul(D, O) - R).T, QC).T
       
        T = matrix(T)
        pred, U_thk = Prediction(H, T, p)
        # print(np.max(np.abs(np.matmul(M, x_kshort) + np.matmul(C, U_thk) - R)[2::3]))
        U_k[:, k] = pred.reshape(p)
        pred = np.expand_dims(pred, axis=1)
        cm_array = np.array(data.geom_xpos[2:6]).mean(axis=0)
        rotation_matrix = data.geom_xmat[1].reshape(3, 3)
        euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
        # euler_angles[2] - np.pi/2, angles[k], 
        # print(euler_angles[2] - np.pi/2 - angles[k])
        X_k[:2, k] = cm_array[:2] - mojoco_origin[:2]
        X_k[2, k] = euler_angles[2] - np.pi/2
        # print(f"cm{cm_array[:2]}  X_k{X_k[:, k]}")
        # print(pred)
        # 计算左轮和右轮的速度
        v_L = U_k[0, k] + (W / 2) * U_k[1, k]
        v_R = U_k[0, k] - (W / 2) * U_k[1, k]
        omega_R, omega_L = v_L / r, v_R / r

        vhc.update_state_linear(omega_R, omega_L, t)
        model_trajectory.append([vhc.state_linear[0], vhc.state_linear[1]])

        # Berechnung der Radgeschwindigkeiten
        wheel_speeds = [omega_L, omega_R, omega_L, omega_R]
        # 控制车轮速度
        control_vel([6, 7, 8, 9], wheel_speeds)

        # 模拟步骤
        for _ in range(simulation_step):
            mujoco.mj_step(model, data)
            
        # 获取当前位置
        geom_pos_12 = data.geom_xpos[12]
        geom_pos_14 = data.geom_xpos[14]
        mid_pos = (geom_pos_12 + geom_pos_14) / 2
        trajectory.append(cm_array)
        
        # # 保持轨迹长度在一定范围内，避免性能问题
        # if len(trajectory) > max_traj_length:
        #     trajectory.pop(0)

        wheel_pos_list.append(np.array(data.geom_xpos[2:6]))

        viewer.render()

        k += 1  # Schrittzähler erhöhen

    wheel_pos = np.array(wheel_pos_list)

    # 关闭Viewer
    viewer.close()

    # 绘制轮子位置（optional）
    plt.figure(figsize=(8, 6))
    for i in range(4):
        x_coords = wheel_pos[:, i, 0]
        y_coords = wheel_pos[:, i, 1]
        plt.scatter(x_coords, y_coords, label=f'Wheel {i+1}', s=50)
    plt.xlabel('X-Koordinate')
    plt.ylabel('Y-Koordinate')
    plt.title('Positionen der Räder')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    # Plotten des geplanten Pfades und der realen Trajektorie
    plt.figure(figsize=(8, 6))
    # Zeichnen des Referenzpfades
    # plt.plot(path_x, path_y, 'k--', label='Geplanter Pfad')
    # Zeichnen der tatsächlichen Trajektorie
    actual_traj = np.array(trajectory)-mojoco_origin
    model_trajectory = np.array(model_trajectory)
    plt.plot(actual_traj[:, 0], actual_traj[:, 1], 'r-', label='Reale Trajektorie')
    # plt.plot(model_trajectory[:, 0], model_trajectory[:, 1], label='model Trajektorie')
    plt.plot(X[N:-N],path[N:-N], 'b--', label='Geplanter Pfad')
    # Markieren der Start- und Endpunkte
    # plt.scatter(path_x[0], path_y[0], color='green', label='Startpunkt', zorder=5)
    # plt.scatter(path_x[-1], path_y[-1], color='blue', label='Endpunkt', zorder=5)
    plt.xlabel('X-Koordinate')
    plt.ylabel('Y-Koordinate')
    plt.title('Vergleich von geplantem Pfad und realer Trajektorie')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()