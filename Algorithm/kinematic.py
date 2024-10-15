import numpy as np
from math import *
import matplotlib.pyplot as plt
import math
from matplotlib.widgets import Slider, Button
import matplotlib
import copy
from matplotlib.animation import FuncAnimation
from scipy.signal import savgol_filter
import infer_func
import torch

L1=0.10513
L2=0.09761
L3=0.17093

def get_all_state(joints):
    x0, y0 = 0, 0
    DH = np.array([
        [joints[0], L1],
        [joints[1], L2],
        [joints[2], L3],
    ])
    DH_tmp = DH
    DH_tmp[0, 0] += np.pi / 2
    pos = np.array([
        [1, 0, x0],
        [0, 1, y0],
        [0, 0, 1]
    ])
    p_list = [np.array([x0, y0, 0])]
    for link in DH_tmp:
        theta = link[0]
        d = link[1]
        # 转换矩阵
        T = np.array([
            [cos(theta), -sin(theta), cos(theta) * d],
            [sin(theta), cos(theta), sin(theta) * d],
            [0, 0, 1]
        ])
        pos = pos @ T
        p_list.append(np.append(pos[:2, 2], [theta]))
    # print(DH[:,0].sum())
    return p_list, DH[:, 0].sum()


def plot_robot(p_list, color="red"):
    # 提取节点坐标的x、y、z值
    x = [p[0] for p in p_list]
    y = [p[1] for p in p_list]
    # nodes = ax.scatter(x, y, z, marker='o')
    fig, ax = plt.subplots(1, 1,dpi=150)

    lines = []
    x_traj, y_traj = [], []
    # 绘制连接线
    for i in range(len(p_list) - 1):
        line, = ax.plot([x[i], x[i + 1]], [y[i], y[i + 1]], color=color)
        lines.append(line)

color1 = next(plt.gca()._get_lines.prop_cycler)['color']

color1 = next(plt.gca()._get_lines.prop_cycler)['color']
color2 = next(plt.gca()._get_lines.prop_cycler)['color']
color3 = next(plt.gca()._get_lines.prop_cycler)['color']
color4 = next(plt.gca()._get_lines.prop_cycler)['color']
color5 = next(plt.gca()._get_lines.prop_cycler)['color']

plt.rcParams['text.usetex'] = True
plt.rc("font",family='Times New Roman')

def wrap_to_2pi(angle):
    """transform the angle to [0, 2pi]"""
    return angle % (2 * np.pi)

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi] range"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def normalize_to_pi(angle_array):
    # 将角度值变化到 -pi 到 pi 之间
    return (angle_array + np.pi) % (2 * np.pi) - np.pi

def get_all_state(joints):
    """
    get the position of all joint and alpha
    """
    x0,y0=0,0
    DH = np.array([
    [joints[0],L1],
    [joints[1],L2],
    [joints[2],L3],
    ])
    DH_tmp=copy.deepcopy(DH)
    DH_tmp[0,0]+=np.pi/2
    pos=np.array([
        [1,0,x0],
        [0,1,y0],
        [0,0,1]
        ])
    p_list=[np.array([x0,y0,0])]
    for link in DH_tmp:
        theta=link[0]
        d=link[1]
        # 转换矩阵
        T=np.array([
            [cos(theta),-sin(theta),cos(theta)*d],
            [sin(theta),cos(theta),sin(theta)*d],
            [0,0,1]
            ])
        pos=pos@T
        p_list.append(np.append(pos[:2,2],[theta]))
    return p_list,np.array(p_list)[:,-1].sum()

def calculate_triangle_angles(a, b, c):
    """
    Calculate the angles of a triangle given its side lengths
    
    Parameters:
    a, b, c: lengths of the three sides of the triangle
    
    Returns:
    A, B, C: 三角形的角度（弧度）
    """
    A = math.acos((b**2 + c**2 - a**2) / (2*b*c))
    B = math.acos((a**2 + c**2 - b**2) / (2*a*c))
    C = math.acos((a**2 + b**2 - c**2) / (2*a*b))
    return A, B, C

def calculate(x, y, theta):
    """
    inverse kinematic in 2D
    """
    x=-x
    global L1,L2,L3
    
    joints=[0,0,0]

    alpha, beta, lp, Bx, By = 0.0, 0.0, 0.0, 0.0, 0.0
    
    Bx = x - L3*math.cos(theta) 
    By = y - L3*math.sin(theta)
    lp = Bx*Bx + By*By
    
    if math.sqrt(lp) >= L1 + L2 or math.sqrt(lp) <= abs(L1 - L2):
        return 1
    
    alpha = math.atan2(By, Bx)
    beta = math.acos((L1*L1 + lp - L2*L2) / (2*L1*math.sqrt(lp)))  # 这里使用弧度制
    
    ptheta = [0.0, 0.0, 0.0]
    ptheta[0] =   alpha -  beta-np.pi/2
    ptheta[1] = -(math.acos((L1*L1 + L2*L2 - lp) / (2*L1*L2)) - math.pi)
    # ptheta[0] =   alpha +  beta-np.pi/2
    # ptheta[1] = (math.acos((L1*L1 + L2*L2 - lp) / (2*L1*L2)) - math.pi)
    ptheta[2] = -ptheta[0] - ptheta[1] + theta-np.pi/2

    ptheta=[wrap_to_pi(theta) for theta in ptheta]
    return ptheta
        
def trans_euler(euler):
    return np.array([np.pi/2-euler[2],euler[1],euler[0]])

def linear_interpolation(arr):
    interpolated_arr = arr.copy()  # 创建一个副本，以免修改原始数组
    rows, _ = arr.shape
    for i in range(rows):
        non_zero_indices = np.nonzero(arr[i])[0]  # 获取非零元素的索引
        # print(non_zero_indices)
        if len(non_zero_indices) < 2:  # 如果一行中只有一个非零元素或没有非零元素，则跳过插值
            left_index = np.max(np.nonzero(arr[:,0][:i]))  # 获取左侧最近的非零元素索引
            # print(left_index)
            right_unzero=np.nonzero(arr[:,0][i:])

            if(len(right_unzero[0])!=0):
                right_index = i+np.min(right_unzero)  # 获取右侧最近的非零元素索引
                # print(right_index)
                left_val = arr[left_index]
                right_val = arr[right_index]
                # 线性插值
                interpolated_arr[i] = left_val + ((right_val - left_val) / (right_index - left_index)) * (i- left_index)
            else:
                interpolated_arr[i]=interpolated_arr[i-1]
    return interpolated_arr

def simulated_annealing(f, x_range, l=1.0, K=1.0, max_accept=10, max_iter=1000):
    N = 1
    accept = 0
    x1 = np.random.uniform(x_range[0], x_range[1])
    best_x = x1
    best_f = f(x1)

    while accept < max_accept and N < max_iter:
        x2 = x1 + l * math.exp(-N / K) * (np.random.rand() - 0.5)
        while x2 < x_range[0] or x2 > x_range[1]:
            x2 = x1 + l * math.exp(-N / K) * (np.random.rand() - 0.5)
        
        f2 = f(x2)
        if f2 < f(x1):
            x1 = x2
            accept += 1
            if f2 < best_f:
                best_x = x2
                best_f = f2
        else:
            if np.random.rand() < 0.3:
                x1 = x2
                accept += 1
            else:
                accept = 0
        
        N += 1
    
    return best_x, best_f

def forward_kinematic(phi,joints):
    DH = np.array([
    [joints[0],L1],
    [joints[1],L2],
    [joints[2],L3],
    ])
    end_effector=get_all_state(DH)[0][-1]
    x = -end_effector[0]
    y = end_effector[1]
    # 绘制连接线
    x,y,z=x*np.cos(phi), x*np.sin(phi),y
    return x,y,z

def inverse_kinematic(path,cur_theta,delta_theta):
    
    p_num=len(path)
    result_list=np.zeros((p_num,3))
    T=np.linspace(0,1,p_num)
    for index in range(p_num):
        t=T[index]
        cur_way_point=path[index]
        # print(cur_way_point)

        x=cur_way_point[0]
        y=cur_way_point[1]
        z=cur_way_point[2]
        cur_theta+=delta_theta
        cur_ans=calculate(sqrt(x**2+y**2),z,cur_theta)
        res=cur_ans
        # print(res)
        if(res!=1 and abs(res[0])>0.2 and abs(res[1])>0.2):
            result_list[index,:]=res
    # print(result_list)
    result_list=linear_interpolation(result_list)
    return result_list

def inverse_kinematic_with_alpha(path,alpha_list):
    
    p_num=len(path)
    result_list=np.zeros((p_num,3))
    T=np.linspace(0,1,p_num)
    for index in range(p_num):
        t=T[index]
        cur_way_point=path[index]
        # print(cur_way_point)

        x=cur_way_point[0]
        y=cur_way_point[1]
        z=cur_way_point[2]
    
        cur_theta=alpha_list[index]

        cur_ans=calculate(sqrt(x**2+y**2),z,cur_theta)            
        res=cur_ans
        if(res!=1):
            result_list[index,:]=res
    print(result_list)
    result_list=linear_interpolation(result_list)
    return result_list

def check_solvability(l, z):
    global L1,L2,L3

    l2 = L1+L2
    l2p = L1-L2
    l3 = L3
    l0=np.linalg.norm([l,z])
    status=[0,0,0,0]

    
    if l0<l2+l3 and l0>abs(l2-l3):
        angle_1,angle_2,angle_3=calculate_triangle_angles(l0,l2,l3)
        angle_0 = np.pi/2 + abs(np.arctan(z/l)) if z < 0 else np.arctan(l/z)
        status[0] = wrap_to_2pi(np.pi - angle_1 + angle_0 - angle_3 + np.pi/2)
        status[1] = wrap_to_2pi( status[0]-2*angle_2)
    if l0<l2p+l3 and l0>abs(l2p-l3):
        angle_1p,angle_2p,angle_3p=calculate_triangle_angles(l0,l2p,l3)
        angle_0 = np.pi/2 + abs(np.arctan(z/l)) if z < 0 else np.arctan(l/z)
        status[2] = wrap_to_2pi(np.pi - angle_1p + angle_0 - angle_3p + np.pi/2)
        status[3] = wrap_to_2pi( status[2]-2*angle_2p)
    return status

def de_casteljau(t, pos_list):
        cur_list = pos_list
        while len(cur_list) != 1:
            tmp_list = []
            for i in range(len(cur_list)-1):
                p = (1-t)*cur_list[i] + t*cur_list[i+1]
                tmp_list.append(p)
            cur_list = tmp_list
            # print(cur_list)
        return cur_list[0]

def generate_filtered_path(point_list, end_d,p_num):
    global L1,L2,L3
    end_ori_euler=[0.03009164 , 0.01929119 ,-0.01088437]
    end_ori_euler=trans_euler(end_ori_euler)
    end_down_euler=np.array([0,0,-1])
    end_down_euler=np.cross(np.cross(end_ori_euler,np.array([0,0,-1])),end_ori_euler)
    end_control_p = point_list[-1] - end_d * end_down_euler

    num_lines=len(point_list)-1
    T_list = np.linspace(0, 1, p_num)
    control_list=[end_control_p]


    for i in range(num_lines-1):
        cur_point=point_list[num_lines-i-1]
        # print(control_list[-1])
        control_list.append(2*cur_point-control_list[-1])

    path = np.zeros((p_num*num_lines, 3))
    for i in range(num_lines):
        # print(i)
        for j in range(p_num):    
            path[p_num*i+j] = de_casteljau(T_list[j], [point_list[i], control_list[num_lines-i-1], point_list[i+1]])
    
    joints = [0, 0, 0]
    DH = np.array([
        [joints[0], L1],
        [joints[1], L2],
        [joints[2], L3],
    ])
    
    cur_theta = np.pi
    target_theta = 1.5*np.pi
    delta_theta = (target_theta - cur_theta) / p_num
    
    result_list = inverse_kinematic(path, cur_theta, delta_theta)
    
    end_effector_pos = []
    for index, data in enumerate(result_list):
        x = path[index][0]
        y = path[index][1]
        dphi = np.arctan(y/x)
        DH[:, 0] = data
        cur_end_pos = forward_kinematic(dphi, DH)
        end_effector_pos.append(cur_end_pos)
    
    end_effector_pos = np.array(end_effector_pos)
    end_effector_pos_new = np.zeros_like(end_effector_pos)
    
    # for i in range(3):
    #     end_effector_pos_new[:, i] = savgol_filter(end_effector_pos[:, i], 50, 2, mode='nearest')
    end_effector_pos_new=end_effector_pos

    return end_effector_pos_new,path,end_effector_pos,control_list

def generate_path(start_pos, end_pos, end_d,p_num=300):
    """
    control point is above the end pos
    """
    global L1,L2,L3
    end_ori_euler=[0.03009164 , 0.01929119 ,-0.01088437]
    end_ori_euler=trans_euler(end_ori_euler)
    end_down_euler=np.array([0,0,-1])
    end_control_p = end_pos - end_d * end_down_euler

    T = np.linspace(0, 1, p_num)
    path = np.zeros((p_num, 3))
    for i in range(p_num):
        path[i] = de_casteljau(T[i], [start_pos,end_control_p, end_pos])
    return path

def plot_segmented(data,color,linewidth):
    x = np.arange(len(data))
    
    # 自动检测非零段
    mask = np.array(data) != 0
    diff = np.diff(np.concatenate(([False], mask, [False])))
    all_changes = np.where(diff > 0)[0]
    start=all_changes[::2]
    end=all_changes[1::2]
    # 为每个段落绘制一条线
    # color = next(plt.gca()._get_lines.prop_cycler)['color']
    segment_list=[]
    for i in range(0):
        cur_x=x[start[i]:end[i]]
        cur_data=data[start[i]:end[i]]
        plt.plot(cur_x, cur_data, color=color,linewidth=linewidth)
        segment_list.append((cur_x,cur_data))
    return segment_list

def plot_angle_analysis(p_num, theta_1_list, theta_2_list, theta_3_list, theta_4_list, tar_theta):
    global color1,color2,color3,color4,color5
    plt.plot(range(p_num), theta_1_list, label=r"$ \alpha_{1}$",linewidth=2,zorder=5)
    plt.plot(range(p_num), theta_2_list, label=r"$ \alpha_{2}$",linewidth=2,zorder=5)
    if 0 in theta_3_list or 0 in theta_4_list:
        segment_list_upper = plot_segmented(theta_3_list, color=color3, linewidth=2)
        segment_list_lower = plot_segmented(theta_4_list, color=color4, linewidth=2)
        for i in range(len(segment_list_upper)):
            upper_x, upper_data = segment_list_upper[i]
            lower_x, lower_data = segment_list_lower[i]
            plt.fill_between(upper_x, upper_data, lower_data, color='white', alpha=1)
    else:
        plt.plot(range(p_num), theta_3_list, color=color3, linewidth=2,label=r"$ \alpha^{'}_{1}$")
        plt.plot(range(p_num), theta_4_list, color=color4, linewidth=2,label=r"$ \alpha^{'}_{2}$")
        # 为对应的线段之间填充白色

    # plt.plot(range(p_num),tar_theta,label=r"Target Angle")
    plt.scatter(0,tar_theta[0],color=color5,label=r"Start Angle")
    plt.scatter(p_num-1,tar_theta[-1],color=color4,label=r"End Angle")
    plt.legend()
    # Plot second solution set


def select_alpha(path,start_theta,end_theta):
    """
    generate alpha for each point in the path
    """
    p_num=len(path)
    tar_theta=[start_theta,end_theta]
    theta_1_list = []
    theta_2_list = []
    theta_3_list = []
    theta_4_list = []
    for index in range(p_num):
        cur_way_point=path[index]
        x=cur_way_point[0]
        y=cur_way_point[1]
        z=cur_way_point[2]
        dphi=np.arctan(y/x)
        index+=1
        l=sqrt(x**2+y**2)
        status = check_solvability(l,z)
        # print(status)
        
        # 将theta值添加到相应的列表中
        theta_1_list.append(status[0])
        theta_2_list.append(status[1])
        theta_3_list.append(status[2])
        theta_4_list.append(status[3])

    print(theta_3_list,theta_4_list)
    plot_angle_analysis(p_num, theta_1_list, theta_2_list, theta_3_list, theta_4_list,  tar_theta)
  
    plt.show()
    theta_3_list2=[None if x == 0 else x for x in theta_3_list]
    theta_4_list2=[None if x == 0 else x for x in theta_4_list]

    upper_limit_list4=max(theta_1_list,theta_2_list)
    lower_limit_list4=min(theta_1_list,theta_2_list)
    upper_limit_list3=max(theta_3_list2,theta_4_list2)
    lower_limit_list3=min(theta_3_list2,theta_4_list2)

    # start_state=int(input("start state:"))
    # end_state=int(input("end state:"))
    start_state=0
    end_state=0
    state_trans=0
    alpha_path=np.zeros(p_num)

    p1=(end_theta-lower_limit_list4[-1])/(upper_limit_list4[-1]-lower_limit_list4[-1])
    print("p1",p1)
    p2=(start_theta-lower_limit_list4[0])/(upper_limit_list4[0]-lower_limit_list4[0])
    p=(p1+p2)/2
    
    for i in range(p_num):
        alpha_path[i]=upper_limit_list4[i]*p+lower_limit_list4[i]*(1-p)
    
    fittered_path=savgol_filter(alpha_path, 25, 2, mode= 'nearest')
    sigma_init=0.5
    X=np.array(fittered_path)
    Var=np.ones_like(X)*sigma_init
    K=np.diag(Var)
    K2=infer_func.init_K(K,10)
    sigma_target=0.001

    C2=infer_func.gen_C(X,-1)
    C1=infer_func.gen_C(X,0)
    mu1=np.array([[start_theta]])
    mu2=np.array([[end_theta]])

    K_N=np.array([[sigma_target]])
    Mu=X[:,np.newaxis]

    K_p=np.linalg.inv(np.linalg.inv(K)+C1@np.linalg.inv(K_N)@C1.T+C2@np.linalg.inv(K_N)@C2.T)
    X_p=Mu+K@(C1@np.linalg.inv(C1.T@K@C1+K_N)@(mu1-C1.T@Mu)+C2@np.linalg.inv(C2.T@K@C2+K_N)@(mu2-C2.T@Mu))

    X_p=Mu+K@C1@np.linalg.inv(np.array([[K[0,0]]])+K_N)@(mu1-C1.T@Mu)+K@C2@np.linalg.inv(np.array([[K[-1,-1]]])+K_N)@(mu2-C2.T@Mu)

    X_p1=X_p.reshape(-1)

    Index=range(len(X))
    diag=np.diag(K_p)
    # color6 = next(plt.gca()._get_lines.prop_cycler)['color']
    # color7 = next(plt.gca()._get_lines.prop_cycler)['color']
    Theta=torch.from_numpy(np.random.multivariate_normal(X_p1,K_p)).requires_grad_()
    Mu=torch.from_numpy(X_p1)
    optimizer = torch.optim.Adam([Theta], lr=0.01)
    K_pv=torch.from_numpy(np.linalg.inv(K_p))

    def reward_func(Theta):
        center_factor=(Theta-Mu)@K_pv@(Theta-Mu).T
        smooth_factor=torch.norm(Theta[1:]-Theta[:-1])
        return center_factor+200*smooth_factor

    for i in range(2000):
        optimizer.zero_grad()
        loss = reward_func(Theta)
        if i % 500 == 0:
            print(f'Loss at iteration {i}: {loss.item()}')
        loss.backward()
        optimizer.step() 

    reslut_alpha=Theta.detach().numpy()

    plot_angle_analysis(p_num, theta_1_list, theta_2_list, theta_3_list, theta_4_list,  tar_theta)
    plt.plot(reslut_alpha)
    plt.show()
    return reslut_alpha