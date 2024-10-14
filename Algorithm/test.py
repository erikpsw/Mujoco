from dm_control import mujoco
import mujoco_viewer
import numpy as np
import kinematic

# 设置模型文件路径
model_path = "../model/mini_mec_six_arm2.xml"

# 加载模型
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

origin_pos = np.array([0.0291812, -0.00117808, 0.1737953])
start_pos = np.array([0.02787045, 0.00068883, 0.51154592]) - origin_pos
end_pos = np.array([0.31583718 ,0.00741404 ,0.0830804 ]) - origin_pos

# path_list=kinematic.generate_path(start_pos,end_pos,end_d=0.05)
# result_list=kinematic.select_alpha(path_list,np.pi/6,1.2+0.6+1.4-0.2)
# angle_list=kinematic.inverse_kinematic_with_alpha(path_list,result_list)
# joint_list=[kinematic.wrap_to_2pi(t)-np.pi for t in angle_list]
# 持续运行模拟
joint_angles = np.array([0., 0., 0., 0, 0., 0,0])
# vel = [1000, 20, 1000, 20]
vel=[0,0,0,0,0,0]
target_get_joint_angles=np.array([0,-1.2, 0.5, 1.5 ,0,0.2,0.2])

d1=200
start=0
end=d1
delta_joints=(target_get_joint_angles-joint_angles)/d1
d2=300
idx=0
times=0

# 初始化轨迹点列表
trajectory = []
max_traj_length = 1000
# 添加一个用于轨迹的线条
viewer.add_line_to_fig('trajectory', 0)  # 添加到第一个图形（索引0）

while True:
    if(start<idx<d1):
        joint_angles+=delta_joints
    # if(d2<idx<d2+300):
    #     if idx%3==0:
    #         i=int((idx-d2)/3)
    #         joint_angles[1:4]=joint_list[i]
        
    control_joints([0,1,2,3,4,5], list(joint_angles))    
    control_vel([6,7,8,9], vel)            
    mujoco.mj_step(model, data)
    idx+=1
    # geom_name = model.geom(14).name
    geom_pos_12 = data.geom_xpos[12]
    geom_pos_14 = data.geom_xpos[14]
    mid_pos = (geom_pos_12 + geom_pos_14) / 2
    print( mid_pos)
    # 添加当前位置到轨迹
    trajectory.append(mid_pos)
    
    # 保持轨迹长度在一定范围内，避免性能问题
    if len(trajectory) > max_traj_length:
        trajectory.pop(0)
        
    # 遍历并绘制整个轨迹
    for pos in trajectory:
        viewer.add_marker(pos=pos, size=np.array([0.001, 0.001, 0.001]), rgba=[1,0, 0, 1])

    viewer.render()

   

