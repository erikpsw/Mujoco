from dm_control import mujoco
import mujoco_viewer
import numpy as np
import matplotlib.pyplot as plt

# 设置模型文件路径
model_path = "../model/mini_mec_six_arm_4wd.xml"
# model_path = "../model/mini_mec_six_arm_cylinder.xml"
# 加载模型
# 

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# 创建渲染器
viewer = mujoco_viewer.MujocoViewer(model, data)

# 定义关节控制函数
def control_joints(joint_ids, positions):
    for joint_id, position in zip(joint_ids, positions):
        data.ctrl[joint_id] = position  # 使用 actuator 来控制关节位置

def control_vel(joint_ids, positions):
    for joint_id, position in zip(joint_ids, positions):
        data.ctrl[joint_id] = position  # 使用 actuator 来控制关节位置

# 持续运行模拟
joint_angles = [0, 0, 0, 0, 0, 0]
vel = [200, 200, 200, 200]
# vel=[0,0,0,0]
duration = 20

# 初始化轨迹点列表
trajectory = []
max_traj_length = 1000
wheel_pos_list=[]

# 添加一个用于轨迹的线条
viewer.add_line_to_fig('trajectory', 0)  # 添加到第一个图形（索引0）
while viewer.is_alive:
    control_joints([0,1,2,3,4,5], joint_angles)    
    control_vel([6,7,8,9], vel)
    mujoco.mj_step(model, data)
    # for i in range(10):           
    #     mujoco.mj_step(model, data)
    geom_name = model.geom(14).name
    # 添加当前位置到轨迹
    geom_pos_12 = data.geom_xpos[12]
    geom_pos_14 = data.geom_xpos[14]
    mid_pos = (geom_pos_12 + geom_pos_14) / 2
    trajectory.append(np.array(mid_pos))
    # joint_angles[1] -= 0.005
    # joint_angles[0] += 0.005
    # joint_angles[2] += 0.005
    # print(joint_angles)
    # 保持轨迹长度在一定范围内，避免性能问题
    if len(trajectory) > max_traj_length:
        trajectory.pop(0)
    
    print(np.array(data.geom_xpos[2:6]))
    wheel_pos_list.append(np.array(data.geom_xpos[2:6]))
    # 遍历并绘制整个轨迹
    for pos in trajectory:
        viewer.add_marker(pos=pos, size=np.array([0.001, 0.001, 0.001]), rgba=[1,0, 0, 1])
    viewer.add_data_to_line('trajectory', mid_pos[0], 0)  # 只使用y坐标作为示例
    viewer.render()

wheel_pos=np.array(wheel_pos_list)

plt.figure(figsize=(6, 6))
for i in range(4):
    x_coords = wheel_pos[:, i, 0]
    y_coords = wheel_pos[:, i, 1]
    plt.scatter(x_coords, y_coords, label=f'Wheel {i+1}', s=100)

# 设置坐标轴和图例
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Visualization of Wheel Positions')
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')

# 显示图像
plt.show()
