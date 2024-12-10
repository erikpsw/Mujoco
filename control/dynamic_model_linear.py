import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import math

class DifferentialDriveRobot:
    def __init__(self):
        # 车轮坐标（忽略z坐标）
        self.wheel_positions = np.array([
            [0.09497593, 0.08389566],
            [-0.07802407, 0.08389566],
            [-0.07802394, -0.08248966],
            [0.09497606, -0.08248966],
        ])
        
        self.local_state = np.array([0.0, 0.0, 0.0])
        self.local_state_linear = np.array([0.0, 0.0, 0.0])
        # 机器人初始状态 [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        self.state_linear = np.array([0.0, 0.0, 0.0])
        
        # 车辆参数
        self.W = 0.6  # 轮距（根据轮子坐标估算）
        
    def update_state(self, vL, vR, dt):
        """
        更新机器人状态
        vL: 左轮速度
        vR: 右轮速度
        dt: 时间步长
        """
        
        theta=self.state[2]
        # 计算ICR
        ICR_x = -self.local_state[1] / self.local_state[2] if abs(theta) > 1e-6 else 0
        ICR_y = self.local_state[0] / self.local_state[2] if abs(theta) > 1e-6 else 0
        
        # 使用矩阵形式更新状态
        v_o = np.array([vL, vR])
        transformation_matrix = np.array([[1/2, 1/2],
                                           [ICR_x/self.W, -ICR_x/self.W],
                                           [1/self.W, -1/self.W]])
        self.local_state = transformation_matrix @ v_o
        # print(self.local_state)
        self.state[0] += self.local_state[0] * np.cos(theta) * dt - self.local_state[1] * np.sin(theta) * dt
        self.state[1] += self.local_state[0] * np.sin(theta) * dt + self.local_state[1] * np.cos(theta) * dt
        self.state[2] += self.local_state[2] * dt
        return ICR_x, ICR_y, np.sqrt(ICR_x**2 + ICR_y**2)

    def update_state_linear(self, vL, vR, dt):
        """
        更新机器人状态
        vL: 左轮速度
        vR: 右轮速度
        dt: 时间步长
        """

        
        # Calculate v and omega
        transformation_matrix = np.array([[1/2, 1/2],
                                        [1/self.W, -1/self.W]])
        v_o = np.array([vL, vR])
        # print(transformation_matrix @ v_o)
        v, omega = transformation_matrix @ v_o
        v += 0.1
        omega += 0.1
        # Update the state using the linearized model
        theta_r = self.state_linear[2]
        A_hat, B_hat, O_hat = compute_ABO(theta_r, v, dt)
        u_k = np.array([v, omega])
        self.state_linear = np.matmul(A_hat, self.state_linear) + np.matmul(B_hat, u_k) + O_hat.flatten()
        # print(self.state_linear)
        # print(self.state)

    
    def get_wheel_positions_world(self):
        """获取世界坐标系中的轮子位置"""
        cos_theta = np.cos(self.state[2])
        sin_theta = np.sin(self.state[2])
        R = np.array([[cos_theta, -sin_theta],
                     [sin_theta, cos_theta]])
        
        world_positions = []
        for wheel_pos in self.wheel_positions:
            pos = R @ wheel_pos + self.state[:2]
            world_positions.append(pos)
        
        return np.array(world_positions)

def compute_ABO(theta_r, v_r, t):
    """
    Compute the matrices for the discrete-time linearized model.
    
    Parameters:
    theta_r: float - Reference orientation
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



def init():
    width=2
    ax.set_xlim(-width, width)
    ax.set_ylim(-width, width)
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    return []

trajectory = []  # Define a trajectory list to store the robot's path
trajectory2 = []  # Define another trajectory list to store the robot's path using update2

def animate(frame):
    # 创建动画
    ax.clear()  
    init()
    dt = 0.01
    # Simulate different motion modes
    t = frame * 0.1
    vL = 1  # Left wheel speed
    vR = 0.5  # Right wheel speed

    # Update robot state
    ICR_x, ICR_y, R_ICR = robot.update_state(vL, vR, dt)
    
    # Record the current position in the trajectory
    trajectory.append((robot.state[0], robot.state[1]))
    # Update robot state
    robot.update_state_linear(vL, vR, dt)
    # Record the current position in the second trajectory using update2
    trajectory2.append((robot.state_linear[0], robot.state_linear[1]))  # Example update2 logic
    print(robot.state, robot.state_linear)
    # Draw the robot body
    wheel_positions = robot.get_wheel_positions_world()
    
    # Draw wheels
    for wheel_pos in wheel_positions:
        ax.plot(wheel_pos[0], wheel_pos[1], 'ko', markersize=5)
    
    # Draw the robot's body outline
    hull = plt.Polygon(wheel_positions, fill=False, color='blue')
    ax.add_patch(hull)
    
    # Draw the robot's orientation
    heading_length = 0.2
    ax.arrow(robot.state[0], robot.state[1],
             heading_length * np.cos(robot.state[2]),
             heading_length * np.sin(robot.state[2]),
             head_width=0.05, head_length=0.05, fc='r', ec='r')
    
    # Draw the robot's trajectory
    trajectory_x, trajectory_y = zip(*trajectory)  # Unzip the trajectory list
    ax.plot(trajectory_x, trajectory_y, 'r-', markersize=2, label = "Nonelinear Trajectory")  # Draw red trajectory
    
    # Draw the second trajectory
    trajectory2_x, trajectory2_y = zip(*trajectory2)  # Unzip the second trajectory list
    ax.plot(trajectory2_x, trajectory2_y, 'b--', markersize=2, label='Linear Trajectory')  # Draw blue dashed trajectory
    
    # If ICR exists, draw the ICR point and arc
    if abs(R_ICR) > 1e-6:
        ICR_world_x = robot.state[0] + ICR_x * np.cos(robot.state[2]) - ICR_y * np.sin(robot.state[2])
        ICR_world_y = robot.state[1] + ICR_x * np.sin(robot.state[2]) + ICR_y * np.cos(robot.state[2])
        ax.plot(ICR_world_x, ICR_world_y, 'rx', markersize=10, label='ICR')
    ax.legend()
    return []

if __name__ == '__main__':
    robot = DifferentialDriveRobot()
    # 创建动画
    fig, ax = plt.subplots(figsize=(10, 10))
    anim = FuncAnimation(fig, animate, init_func=init,
                        frames=100, interval=10, blit=True)

    plt.show()