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
            [0.09497606, -0.08248966],
            [-0.07802394, -0.08248966]
        ])
        
        self.local_state = np.array([0.0, 0.0, 0.0])
        # 机器人初始状态 [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        
        # 车辆参数
        self.W = 0.173  # 轮距（根据轮子坐标估算）
        
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
        print(self.local_state)
        self.state[0] += self.local_state[0] * np.cos(theta) * dt - self.local_state[1] * np.sin(theta) * dt
        self.state[1] += self.local_state[0] * np.sin(theta) * dt + self.local_state[1] * np.cos(theta) * dt
        self.state[2] += self.local_state[2] * dt
        return ICR_x, ICR_y, np.sqrt(ICR_x**2 + ICR_y**2)
    
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

# 创建动画
fig, ax = plt.subplots(figsize=(10, 10))
robot = DifferentialDriveRobot()

def init():
    width=4
    ax.set_xlim(-width, width)
    ax.set_ylim(-width, width)
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    return []

trajectory = []  # Define a trajectory list to store the robot's path

def animate(frame):
    ax.clear()
    init()
    
    # Simulate different motion modes
    t = frame * 0.1
    vL = 1  # Left wheel speed
    vR = 0.9  # Right wheel speed
    
    # Update robot state
    ICR_x, ICR_y, R_ICR = robot.update_state(vL, vR, 0.01)
    
    # Record the current position in the trajectory
    trajectory.append((robot.state[0], robot.state[1]))
    
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
    ax.plot(trajectory_x, trajectory_y, 'r-', markersize=2)  # Draw red trajectory
    
    # If ICR exists, draw the ICR point and arc
    if abs(R_ICR) > 1e-6:
        ICR_world_x = robot.state[0] + ICR_x * np.cos(robot.state[2]) - ICR_y * np.sin(robot.state[2])
        ICR_world_y = robot.state[1] + ICR_x * np.sin(robot.state[2]) + ICR_y * np.cos(robot.state[2])
        ax.plot(ICR_world_x, ICR_world_y, 'rx', markersize=10, label='ICR')
        ax.legend()
    
    return []

# 创建动画
anim = FuncAnimation(fig, animate, init_func=init,
                    frames=100, interval=10, blit=True)
plt.show()