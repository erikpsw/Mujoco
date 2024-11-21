from dm_control import mujoco
import mujoco_viewer
import numpy as np
import kinematic

# 设置模型文件路径
model_path = "../model/mini_mec_six_arm_4wd.xml"

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


# origin_pos = np.array([0.05857364, -0.00254144,  0.1743498])
# start_pos = np.array([0.02787045 ,0.00068883, 0.4154592]) - origin_pos
# end_pos = np.array([0.24 ,-3.17631695e-05 , 9.82053450e-02 ]) - origin_pos

# path_list=kinematic.generate_path(start_pos,end_pos,end_d=0.03)
# result_list=kinematic.select_alpha(path_list,np.pi/2,1.2+0.5+1.5+np.pi/2)

# angle_list=kinematic.inverse_kinematic_with_alpha(path_list,result_list)

# def normalize_to_pi(angle_array):
#     # 将角度值变化到 -pi 到 pi 之间
#     return (angle_array + np.pi) % (2 * np.pi) - np.pi
# joint_list=angle_list
# joint_list=normalize_to_pi(angle_list)
# 持续运行模拟
joint_angles = np.array([0., -np.pi/3.,  np.pi/3, 0, 0., 0])
# vel = [1000, 20, 1000, 20]
vel=[0,0,0,0,0,0]
target_get_joint_angles=np.array([0,-1, 0.8, 1.4,0,0])

d1=400
start=0
end=d1
delta_joints=(target_get_joint_angles-joint_angles)/d1
d2=250
idx=0
times=0

# 初始化轨迹点列表
trajectory = []
max_traj_length = 1000
# 添加一个用于轨迹的线条
viewer.add_line_to_fig('trajectory', 0)  # 添加到第一个图形（索引0）

while True:
    if(d2<idx<d2+500):
        
        joint_angles[1:4]=joint_list[i]
        joint_angles[1]*=-1
    print(idx,joint_angles)
        
    control_joints([0,1,2,3,4,5], list(joint_angles))    
    control_vel([6,7,8,9], vel)
    mujoco.mj_step(model, data)
    idx+=1
    # geom_name = model.geom(14).name
    geom_pos_12 = data.geom_xpos[12]
    geom_pos_14 = data.geom_xpos[14]
    mid_pos = (geom_pos_12 + geom_pos_14) / 2
    print(mid_pos)
    j2=data.geom_xpos[8]
    # 获得关节角度
    # print(np.linalg.norm(j2-mid_pos))
    # print(mid_pos)
    trajectory.append(list(mid_pos))
    
    # 保持轨迹长度在一定范围内，避免性能问题
    if len(trajectory) > max_traj_length:
        trajectory.pop(0)
        
    # 遍历并绘制整个轨迹
    for pos in trajectory:
        viewer.add_marker(pos=pos, size=np.array([0.001, 0.001, 0.001]), rgba=[1,0, 0, 1])

    viewer.render()

   

