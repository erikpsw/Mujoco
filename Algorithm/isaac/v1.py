# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.ui as ui
from omni.isaac.examples.base_sample import BaseSample
import numpy as np
from omni.isaac.core.world import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.isaac.core.objects as obj
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.prims import XFormPrim
import math
from math import * 
import asyncio
import uuid
import matplotlib.pyplot as plt
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.rotations import quat_to_euler_angles
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera
from PIL import Image
import csv
import copy
from scipy.signal import savgol_filter

rand_x=0.22
rand_pos_range=0.03
file_path="E:/Program Files/ominverse/isaac_sim-2023.1.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/control_signal.csv"
factor=10000

def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi] range"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

is_wrap=True

def read_headers(file_path):

    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        headers = next(reader)
        initial_values = next(reader)
        return {header: float(value) for header, value in zip(headers, initial_values)}

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

joints=[0,0,0]
DH = np.array([
    [joints[0],0.10513],
    [joints[1],0.09761],
    [joints[2],0.17093],
    ])

def calculate(x, y, theta):
    x=-x
    L1 = DH[0,1]  # 杆长
    L2 = DH[1,1]
    L3 = DH[2,1]
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
    if(ptheta[0]<0):
        return 1
    ptheta[1] = -(math.acos((L1*L1 + L2*L2 - lp) / (2*L1*L2)) - math.pi)
    # ptheta[0] =   alpha +  beta-np.pi/2
    # ptheta[1] = (math.acos((L1*L1 + L2*L2 - lp) / (2*L1*L2)) - math.pi)
    ptheta[2] = -ptheta[0] - ptheta[1] + theta-np.pi/2
    if is_wrap:
        ptheta=[wrap_to_pi(theta) for theta in ptheta]
    return ptheta

def get_all_state(DH):
    x0,y0=0,0
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

async def example(world):
        
        world.scene.add_default_ground_plane()
        global robot_view
        
        # add franka articulations
        # asset_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        asset_path=r'file:/E:/programming/project/ros/urdf/mini_mec_six_arm/mini_mec_six_arm0.usd'
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/robot")

        asset_path2=r'file:/E:/programming/project/ros/urdf/mini_mec_six_arm/block2.usd'
        block_grasp=add_reference_to_stage(usd_path=asset_path2, prim_path="/World/block")
        
        add_reference_to_stage(usd_path=asset_path2, prim_path="/World/block2")
        # world.scene.add(
        #     DynamicCuboid(Th
        #         prim_path="/World/random_cube", # e prim path of the cube in the USD stage
        #         name="fancy_cube", # The unique name used to retrieve the object from the scene later on
        #         position=np.array([rand_x+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.0]), # Using the current stage units which is in meters by default.
        #         orientation=rand_ori,
        #         scale=np.array([0.05015, 0.05015, 0.05015]), # most arguments accept mainly numpy arrays.
        #         color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        #     ))
        robot_view = ArticulationView(prim_paths_expr="/World/robot", name="robot_view")
        world.scene.add(robot_view)
        
        # material = PhysicsMaterial(
        #     prim_path="/World/physics_material/aluminum",  # path to the material prim to create
        #     dynamic_friction=5,
        #     static_friction=5,
        #     restitution=0.5
        # )
        # block_grasp.apply_physics_material(material)
        await world.reset_async()
        # set root body poses
        new_positions = np.array([[0, 0, 0]])
        robot_view.set_world_poses(positions=new_positions)

def trans_euler(euler):
    return np.array([np.pi/2-euler[2],euler[1],euler[0]])

def de_casteljau(t,pos_list):
    cur_list=pos_list
    while len(cur_list)!=1:
        tmp_list=[]
        for i in range(len(cur_list)-1):
            p=(1-t)*cur_list[i]+t*cur_list[i+1]
            tmp_list.append(p)
        cur_list=tmp_list
    return cur_list[0]

def reward_func(target_pos,target_ori,cur_block_pos,cur_block_ori,init_target_pos,init_target_ori):
    if(np.linalg.norm(target_pos-init_target_pos)+np.linalg.norm(target_ori-init_target_ori))<0.2:
        value=-(np.linalg.norm(cur_block_pos-target_pos)+np.linalg.norm(cur_block_ori-target_ori))+0.8
    else:
        value=-1
    return max(-1, min(1, value))

def forward_kinematic(phi,DH):
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
        print(cur_way_point)

        x=cur_way_point[0]
        y=cur_way_point[1]
        z=cur_way_point[2]
        cur_theta+=delta_theta

        cur_ans=calculate(sqrt(x**2+y**2),z,cur_theta)
        res=cur_ans
        print(res)
        if(res!=1 and abs(res[0])>0.2 and abs(res[1])>0.2):
            result_list[index,:]=res
        # print(result_list)
    result_list=linear_interpolation(result_list)
    return result_list

class SingleRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.joint_state=np.zeros(15)
        return

    def setup_scene(self):

        world = self.get_world()
        world.scene.add_default_ground_plane()
        asyncio.ensure_future(example(world))
        return

    async def setup_post_load(self):
        self.joint_state=np.zeros(15)
        self.frame=0
        self._world = self.get_world()
        self.delta_theta=np.zeros(3)
        self._robot_view = self._world.scene.get_object("robot_view")
        self.dc=_dynamic_control.acquire_dynamic_control_interface()
        self.block2_dx=0
        self.block2_dy=0
        self.block2_theta=0
        self.reward_list=[]
        self.path=0
        # self.box=self.dc.get_rigid_body("/World/random_cube")
        self.block=self.dc.get_rigid_body("/World/block/block")
        self.block2=self.dc.get_rigid_body("/World/block2/block")
        robot_view = ArticulationView(prim_paths_expr="/World/block", name="block")
        self._world.scene.add(robot_view)
        self.joint_state[11]=np.pi/6
        self.joint_state[12]=np.pi/6
        self.camera = Camera(
            prim_path="/World/robot/link4/Camera",
            frequency=20,
            resolution=(256, 256),
        )
        # self.window = ui.Window("Plot Example", width=600, height=400)
        self.camera.initialize()
        # self.camera.add_motion_vectors_to_frame()
        print("load")
        np.random.seed(42)
        new_location = np.array([rand_x+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.0])  

        rand_theta=(np.random.random()-0.5)*(np.pi/3)
        
        rand_ori=euler_angles_to_quat(np.array([rand_theta,0,0]))
        
        # Create a new transform
        new_transform = _dynamic_control.Transform(new_location,rand_ori)
        # Apply the new transform
        self.dc.set_rigid_body_pose(self.block, new_transform)
        # create a rigid body physical material
        
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        self.joint_state=np.zeros(15)
        return

    def send_robot_actions(self, step_size):
        # 放置时的偏移
        delta_x=0.02
        delta_y=0.03
        delta_z=0.15
        
        # 抓取时的偏移    
        delta_z2=0.0115
        delta_h=0
        
        mode=1
        self.frame+=1
        print(self.frame)
        robot_view = self._world.scene.get_object("robot_view")
        self.origin=self.dc.get_rigid_body("/World/robot/link2")
        self.effector=self.dc.get_rigid_body("/World/robot/link4")
        box_pose=self.dc.get_rigid_body_pose(self.block)
        # print(quat_to_euler_angles(box_pose.r))
        if mode==1:
            t1=70
            t11=100
            t2=150
            t3=250
            place_time=300
            t4=t3+place_time
            t5=t4+100
            t6=t5+100
            t7=700
            sol_list=np.array([[0.53913432, 0.75940006, 0.81393174],
       [0.53216821, 0.773002  , 0.81249592],
       [0.52465743, 0.78808043, 0.80968122],
       [0.51802696, 0.80109072, 0.80846271],
       [0.51085936, 0.81555541, 0.80588068],
       [0.50463679, 0.82778161, 0.80506392],
       [0.49784836, 0.84153013, 0.80282409],
       [0.4919549 , 0.85315785, 0.80227222],
       [0.48551697, 0.86625091, 0.80034335],
       [0.47996855, 0.87723189, 0.80011522],
       [0.47388158, 0.8896595 , 0.79852431],
       [0.46873484, 0.89984586, 0.79876451],
       [0.46303078, 0.91152147, 0.79760406],
       [0.45817251, 0.92119048, 0.79808675],
       [0.45272659, 0.93241947, 0.79710492],
       [0.44809225, 0.94172728, 0.79770423],
       [0.44291148, 0.95248758, 0.79693087],
       [0.43852885, 0.96136147, 0.79771953],
       [0.43355743, 0.97178759, 0.79704586],
       [0.42938826, 0.98031929, 0.79795117],
       [0.42463838, 0.9903775 , 0.79741487],
       [0.42071409, 0.99848687, 0.79851498],
       [0.41622405, 1.0080802 , 0.79820988],
       [0.41250082, 1.01587371, 0.79941712],
       [0.4082277 , 1.02510608, 0.79925805],
       [0.40469336, 1.03261189, 0.80055301],
       [0.40061258, 1.04154183, 0.8004938 ],
       [0.3972841 , 1.04871698, 0.80191998],
       [0.39339925, 1.05733418, 0.80197526],
       [0.39025129, 1.06423959, 0.80348293],
       [0.38654622, 1.07258211, 0.8036235 ],
       [0.38360268, 1.07915892, 0.8052697 ],
       [0.38010978, 1.08714751, 0.80557126],
       [0.3773308 , 1.09349201, 0.8072735 ],
       [0.37400772, 1.10122853, 0.8076487 ],
       [0.37139695, 1.10733119, 0.809419  ],
       [0.36824265, 1.11481743, 0.80986987],
       [0.36578848, 1.12070553, 0.81168685],
       [0.36280734, 1.12793049, 0.81222668],
       [0.360518  , 1.13358394, 0.81411138],
       [0.3576742 , 1.14063244, 0.81466676],
       [0.35555538, 1.14603755, 0.81663427],
       [0.35289652, 1.15279632, 0.81731027],
       [0.35092415, 1.1580101 , 0.81931164],
       [0.34841254, 1.16456814, 0.82003057],
       [0.34658668, 1.16959005, 0.82206859],
       [0.34422116, 1.1759497 , 0.82283076],
       [0.34254154, 1.18078004, 0.82490729],
       [0.34031628, 1.18695451, 0.82570304],
       [0.33877132, 1.19161994, 0.82779603],
       [0.33667611, 1.19763243, 0.82860646],
       [0.33528297, 1.20209282, 0.83075402],
       [0.33334481, 1.20788068, 0.83163773],
       [0.33207674, 1.21219764, 0.83378598],
       [0.33026571, 1.21782958, 0.83468272],
       [0.32912945, 1.221987  , 0.83684787],
       [0.32744863, 1.22745545, 0.83776624],
       [0.32643908, 1.23146454, 0.83993974],
       [0.32488154, 1.23678473, 0.84086763],
       [0.32399241, 1.24065911, 0.84303871],
       [0.32254907, 1.24585088, 0.84395978],
       [0.32179821, 1.24954974, 0.84616669],
       [0.32050037, 1.25454225, 0.84714632],
       [0.31985535, 1.25813803, 0.84932508],
       [0.31866884, 1.26300745, 0.85029599],
       [0.31814791, 1.26645845, 0.85248552],
       [0.31707809, 1.27119208, 0.85346053],
       [0.31667126, 1.27451986, 0.85564267],
       [0.3157115 , 1.27913161, 0.85661055],
       [0.31541416, 1.28234542, 0.85877823],
       [0.31455747, 1.28684976, 0.85972726],
       [0.31438155, 1.2899225 , 0.86190603],
       [0.31365233, 1.29426518, 0.86288618],
       [0.31357971, 1.29723574, 0.86504223],
       [0.31295251, 1.3014716 , 0.8660054 ],
       [0.31298607, 1.3043326 , 0.86814651],
       [0.31246438, 1.30845301, 0.8691016 ],
       [0.3126016 , 1.31120876, 0.87122501],
       [0.31217931, 1.31522586, 0.87216227],
       [0.3124141 , 1.31788827, 0.87425842],
       [0.31208357, 1.3218172 , 0.87516542],
       [0.31242717, 1.32436091, 0.8772581 ],
       [0.31221139, 1.32815146, 0.87818086],
       [0.31264944, 1.33060599, 0.88024468],
       [0.3125294 , 1.33429759, 0.88114896],
       [0.3130602 , 1.3366651 , 0.88318308],
       [0.31302985, 1.34026919, 0.88405973],
       [0.31365499, 1.3425448 , 0.88606955],
       [0.31372047, 1.34604712, 0.88693235],
       [0.31443709, 1.34823537, 0.88891485],
       [0.31458094, 1.351671  , 0.88973356],
       [0.31539193, 1.3537643 , 0.89169656],
       [0.31564098, 1.35707581, 0.89252337],
       [0.31653156, 1.35910336, 0.89444187],
       [0.31686354, 1.36233568, 0.8952378 ],
       [0.31784379, 1.36427484, 0.897133  ],
       [0.31826342, 1.36741659, 0.89790917],
       [0.31933424, 1.36926389, 0.89978516],
       [0.31982992, 1.37233735, 0.90052281],
       [0.32097953, 1.37411523, 0.90236068],
       [0.32154629, 1.37712872, 0.90305347],
       [0.32277815, 1.3788282 , 0.90486225],
       [0.32343893, 1.38173319, 0.90555422],
       [0.32474039, 1.38337816, 0.90731377],
       [0.32548134, 1.38620122, 0.90798238],
       [0.32686291, 1.38776835, 0.90971439],
       [0.32767857, 1.39051874, 0.91035245],
       [0.32914364, 1.39199935, 0.91206563],
       [0.33002633, 1.39469065, 0.91266218],
       [0.33156107, 1.39611059, 0.91433472],
       [0.33250429, 1.39875379, 0.91488111],
       [0.33411196, 1.4001044 , 0.91652175],
       [0.3351372 , 1.40265452, 0.91705869],
       [0.33681147, 1.40394639, 0.91865913],
       [0.33790569, 1.40642761, 0.91916634],
       [0.33964733, 1.40765704, 0.9207309 ],
       [0.34081005, 1.41006828, 0.92121036],
       [0.34262242, 1.4112265 , 0.92274784],
       [0.34385536, 1.41356242, 0.9232053 ],
       [0.345722  , 1.41467976, 0.92469009],
       [0.34699563, 1.41699639, 0.92507735],
       [0.34893129, 1.41804144, 0.92653818],
       [0.35027992, 1.42026914, 0.92691752],
       [0.35227582, 1.42125701, 0.92834219],
       [0.35368814, 1.42341593, 0.92869694],
       [0.35573967, 1.42435336, 0.93008065],
       [0.3572053 , 1.42646169, 0.93039595],
       [0.35931887, 1.42733387, 0.93175278],
       [0.36083992, 1.42938527, 0.93203527],
       [0.36300974, 1.43020118, 0.93335846],
       [0.36457187, 1.43222127, 0.93358682],
       [0.36678935, 1.43299522, 0.93486499],
       [0.36841724, 1.43493391, 0.93508414],
       [0.370688  , 1.43565267, 0.93633   ],
       [0.37236515, 1.43753972, 0.936515  ],
       [0.37468901, 1.4382013 , 0.93773142],
       [0.37641696, 1.44003152, 0.93788792],
       [0.37879225, 1.4406368 , 0.93907524],
       [0.38056593, 1.44241772, 0.93919779],
       [0.38298574, 1.44297762, 0.94034764],
       [0.38479266, 1.4447309 , 0.94041847],
       [0.38725417, 1.44524839, 0.94152943],
       [0.3891144 , 1.44693326, 0.94158521],
       [0.39161884, 1.44740363, 0.94266256],
       [0.39352259, 1.44903662, 0.94269002],
       [0.39606656, 1.44946396, 0.94373134],
       [0.39801622, 1.45103828, 0.94373753],
       [0.40060712, 1.45140612, 0.94475833],
       [0.4025971 , 1.45293018, 0.94473705],
       [0.40522733, 1.45325066, 0.94572793],
       [0.40723426, 1.45476656, 0.9456437 ],
       [0.40989366, 1.45505653, 0.94659121],
       [0.41194695, 1.45650616, 0.94649574],
       [0.4146429 , 1.45674926, 0.94741525],
       [0.41673284, 1.45814881, 0.94729561],
       [0.41946539, 1.45834262, 0.94819044],
       [0.42159007, 1.4596935 , 0.94804655],
       [0.42435704, 1.45983981, 0.94891629],
       [0.42651593, 1.46114079, 0.94875038],
       [0.42931611, 1.4612396 , 0.94959622],
       [0.43148549, 1.46253318, 0.94937253],
       [0.43430299, 1.46261204, 0.95017176],
       [0.43651085, 1.46384324, 0.94993919],
       [0.43935795, 1.46387675, 0.95071504],
       [0.44159578, 1.46505946, 0.9504627 ],
       [0.44447286, 1.46504462, 0.95121895],
       [0.44673849, 1.46618077, 0.95094631],
       [0.44964509, 1.46611624, 0.9516853 ],
       [0.45193516, 1.46720999, 0.95138991],
       [0.45486609, 1.46710327, 0.95210631],
       [0.4571602 , 1.46819072, 0.95175804],
       [0.46009937, 1.4680697 , 0.95242886],
       [0.46242365, 1.46909977, 0.95207314],
       [0.46538612, 1.46893404, 0.95272593],
       [0.46773475, 1.46991556, 0.95235625],
       [0.47071979, 1.46970436, 0.95299285],
       [0.47308989, 1.47064067, 0.95260748],
       [0.47609831, 1.47038036, 0.95323219],
       [0.47848691, 1.47127498, 0.95282922],
       [0.48151422, 1.47097185, 0.95343776],
       [0.48390749, 1.4718486 , 0.95299769],
       [0.48692683, 1.47155071, 0.95354958],
       [0.48933902, 1.47238101, 0.95309831],
       [0.49237618, 1.47203814, 0.95363823],
       [0.49479962, 1.47283432, 0.95316621],
       [0.49785714, 1.47243972, 0.95370116],
       [0.50030263, 1.47317972, 0.95322864],
       [0.50337821, 1.4727357 , 0.95375774],
       [0.50583477, 1.47343813, 0.95326971],
       [0.50892657, 1.47294614, 0.95379285],
       [0.51136852, 1.47365702, 0.95325055],
       [0.5144507 , 1.47316339, 0.95372874],
       [0.51691464, 1.47381262, 0.95319401],
       [0.52000529, 1.47328165, 0.95365927],
       [0.52247825, 1.47389161, 0.95311382],
       [0.52558448, 1.4733082 , 0.95358036],
       [0.52807052, 1.47386955, 0.95303338],
       [0.5311903 , 1.47323568, 0.95350057],
       [0.53368356, 1.47375758, 0.95294521],
       [0.53681184, 1.47308085, 0.95340757],
       [0.5392852 , 1.47361225, 0.95280248],
       [0.54239513, 1.47294103, 0.95321918],
       [0.54488313, 1.47341577, 0.95262275],
       [0.54800196, 1.47269746, 0.95304044],
       [0.5504991 , 1.47312419, 0.95244623],
       [0.55362587, 1.4723589 , 0.95286587],
       [0.55612969, 1.47274058, 0.95227237],
       [0.55926312, 1.47192904, 0.95269436],
       [0.56177981, 1.47225248, 0.95211405],
       [0.56492179, 1.47138955, 0.9525439 ],
       [0.56740323, 1.47174318, 0.95190112],
       [0.57051955, 1.47089121, 0.95228581],
       [0.57301115, 1.47118869, 0.95165643],
       [0.5761337 , 1.47028654, 0.95204996],
       [0.57863261, 1.47053166, 0.95143167],
       [0.581761  , 1.46957854, 0.95183574],
       [0.58426248, 1.46977872, 0.95122312],
       [0.58738982, 1.46878602, 0.95162883],
       [0.58990539, 1.46891826, 0.95104281],
       [0.59304385, 1.46786173, 0.95147214],
       [0.59551469, 1.4680349 , 0.95081925],
       [0.59862005, 1.46699606, 0.95120247],
       [0.60109958, 1.46710869, 0.95057164],
       [0.60420802, 1.46601837, 0.95096969],
       [0.60669154, 1.46607795, 0.95035538],
       [0.6098048 , 1.4649315 , 0.95077321],
       [0.61229066, 1.46493984, 0.95017478],
       [0.61540691, 1.46373936, 0.95061149],
       [0.61789717, 1.46369128, 0.95003433],
       [0.62101616, 1.46243596, 0.95049151],
       [0.62346173, 1.46242265, 0.94985738],
       [0.6265365 , 1.46119966, 0.95026025],
       [0.62899343, 1.4611145 , 0.94966254],
       [0.63207635, 1.4598241 , 0.95009853],
       [0.63453532, 1.45968347, 0.94952391],
       [0.63762487, 1.45832731, 0.94999244],
       [0.64010437, 1.4580951 , 0.94947309],
       [0.64316316, 1.45674259, 0.94991472],
       [0.64562478, 1.45649019, 0.9493895 ],
       [0.6487174 , 1.45501828, 0.94991187],
       [0.65112948, 1.45480451, 0.94933051],
       [0.65417544, 1.45336423, 0.94980329],
       [0.65659165, 1.45308677, 0.94925541],
       [0.65964062, 1.45158321, 0.94976179],
       [0.66206257, 1.45123791, 0.94925179],
       [0.66511538, 1.44966848, 0.94979476],
       [0.66753479, 1.44927002, 0.94931066],
       [0.67058694, 1.44764221, 0.94988445],
       [0.67303224, 1.44713568, 0.94947469],
       [0.67607795, 1.44545948, 0.95007132],
       [0.67844587, 1.44503999, 0.94956716],
       [0.68145385, 1.44337405, 0.95013642],
       [0.68383164, 1.44287518, 0.94968309],
       [0.68683762, 1.44115075, 0.95028536],
       [0.68922131, 1.44057914, 0.94987778],
       [0.69223864, 1.43876993, 0.95053655],
       [0.69462241, 1.43813574, 0.95016659],
       [0.69764487, 1.43625273, 0.95087293],
       [0.70003153, 1.43554976, 0.9505466 ],
       [0.70305179, 1.43360606, 0.95128974],
       [0.70537639, 1.43295756, 0.95090067],
       [0.70834913, 1.43103901, 0.95160669],
       [0.71071093, 1.43025479, 0.9513206 ],
       [0.71365975, 1.42831574, 0.95202972],
       [0.71599512, 1.42751666, 0.95174227],
       [0.71897475, 1.42545173, 0.9525465 ],
       [0.72132047, 1.42456671, 0.95231996],
       [0.7243088 , 1.42241744, 0.95318373],
       [0.72665648, 1.42146182, 0.95300534],
       [0.72965026, 1.41923373, 0.95392433],
       [0.73193548, 1.41833046, 0.95368796],
       [0.73487051, 1.41614615, 0.95455636],
       [0.73716044, 1.41516595, 0.95437465],
       [0.74010062, 1.41290232, 0.9552997 ],
       [0.74241384, 1.41180882, 0.95520464],
       [0.74536735, 1.40944952, 0.95620088],
       [0.74767584, 1.40829594, 0.95614678],
       [0.75064097, 1.40584369, 0.95721223],
       [0.75295768, 1.40460453, 0.9572216 ],
       [0.75593255, 1.40206235, 0.95835396],
       [0.75817842, 1.4008895 , 0.95829572],
       [0.76109266, 1.39839266, 0.95937831],
       [0.76334991, 1.39712735, 0.95939031],
       [0.76627789, 1.39453204, 0.96054795],
       [0.7685537 , 1.39315988, 0.96064295],
       [0.77150254, 1.3904519 , 0.96188814],
       [0.77379813, 1.38896997, 0.96206896],
       [0.77675905, 1.38616578, 0.96338765],
       [0.77906517, 1.3845915 , 0.96363948],
       [0.78202936, 1.38170775, 0.96501745],
       [0.78425901, 1.38020915, 0.96519554],
       [0.78714138, 1.37741033, 0.96649121],
       [0.78940446, 1.3757745 , 0.9667796 ],
       [0.79227502, 1.37292489, 0.96811033],
       [0.79453046, 1.37123102, 0.96844092],
       [0.79744211, 1.36822778, 0.96989547],
       [0.79970997, 1.36643662, 0.97030224],
       [0.80261312, 1.36337563, 0.97179807],
       [0.80493821, 1.36139988, 0.97235624],
       [0.80790887, 1.35813325, 0.97402049]])
            # print(self.block_prim.get_default_state().position)
            origin_pos=np.array(self.dc.get_rigid_body_pose(self.origin).p)
            if(self.frame<t1):
                dx=box_pose.p[0]-origin_pos[0]-delta_x
                dy=box_pose.p[1]-origin_pos[1] # 原点的偏移
                theta=-atan(dy/dx)
                box_theta=quat_to_euler_angles(box_pose.r)[0]
                dz=box_pose.p[2]-origin_pos[2]+delta_h
                # print(f"{sqrt(dx**2+dy**2)}, {dz}")
                self.res=calculate(sqrt(dx**2+dy**2),dz,np.pi*1.5)
                print(f'dx {dx} dz {dz} ans {self.res}')
                print(dx,dy,dz)
                if(self.res!=1):
                    self.res[0]=-self.res[0]
                    self.joint_state[0]=theta
                    self.joint_state[5]+=(self.res[0]-self.joint_state[5])/50
                    self.joint_state[6]+=(self.res[1]-self.joint_state[6])/50
                    self.joint_state[7]+=(self.res[2]-self.joint_state[7])/50
                    print(f"{theta}  {box_theta}")
                    self.joint_state[8]=theta+box_theta
                else:
                    self.frame=t5-1  
            elif(self.frame==t1):
                self.delta_theta=(self.res-self.joint_state[5:8])/(t11-t1)
            elif(t1<self.frame<t11):
                self.joint_state[5:8]+=self.delta_theta
            elif(self.frame==t11):
                self.joint_state[5]=self.res[0]
                self.joint_state[6]=self.res[1]
                self.joint_state[7]=self.res[2]
                # print(f"here is {self.joint_state[11:13]}")
                self.uuid=uuid.uuid1().__str__()[:23]
            elif(t1<self.frame<t2):
                self.joint_state[11]-=np.pi/150
                self.joint_state[12]-=np.pi/150
            elif(self.frame==t2):
                
                # self.aim_dx=box_pose.p[0]-origin_pos[0]-0.03
                # self.aim_dz=box_pose.p[2]-origin_pos[2]+delta_h
                # self.aim_dy=box_pose.p[1]-(origin_pos[1]-0.0234)
                self.aim=np.array([-np.pi/6,np.pi/4,np.pi/4])
                self.delta_theta=self.aim-self.joint_state[5:8]
            elif(t2<self.frame<t3):
                # self.aim_dz+=0.0005
                # res=calculate(sqrt(self.aim_dx**2+self.aim_dy**2),self.aim_dz,np.pi*1.5) #抓取位置
                # if(res!=1):
                #     self.joint_state[5]=-res[0]
                #     self.joint_state[6]=res[1]
                #     self.joint_state[7]=res[2]
                
                self.joint_state[5:8]+=self.delta_theta/(t3-t2)
            elif(self.frame==t3):
                print("start joint", self.joint_state[5:8])
                dx=box_pose.p[0]-origin_pos[0]-delta_x
                dy=box_pose.p[1]-origin_pos[1] # 原点的偏移
                dz=box_pose.p[2]-origin_pos[2]+delta_h
                print(dx,dy,dz)
            elif(t3+len(sol_list)>self.frame>t3):
                
                self.joint_state[5]=-sol_list[self.frame-t3][0]
                self.joint_state[6]=sol_list[self.frame-t3][1]
                self.joint_state[7]=sol_list[self.frame-t3][2]
        elif(mode==2):
            if(self.frame>t3):
                target_pos=np.array(self.dc.get_rigid_body_pose(self.block2).p)
                target_ori=np.array(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block2).r))
                cur_block_pos=np.array(self.dc.get_rigid_body_pose(self.block).p)
                cur_block_ori=np.array(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block).r))
                # print(f"{target_pos} {type(target_pos)}")
                reward=reward_func(target_pos,target_ori,cur_block_pos,cur_block_ori,self.init_target_pos,self.init_target_ori)
                self.reward_list.append(reward) 
            if(self.frame<t1):
                dx=box_pose.p[0]-origin_pos[0]-delta_x
                dy=box_pose.p[1]-origin_pos[1] # 原点的偏移
                theta=-atan(dy/dx)
                box_theta=quat_to_euler_angles(box_pose.r)[0]
                dz=box_pose.p[2]-origin_pos[2]+delta_h
                print(f"{sqrt(dx**2+dy**2)}, {dz}")
                self.res=calculate(sqrt(dx**2+dy**2),dz,np.pi*1.5)
                # print(f'dx {dx} dz {dz} ans {self.res}')
                
                if(self.res!=1):
                    self.res[0]=-self.res[0]
                    self.joint_state[0]=theta
                    self.joint_state[5]+=(self.res[0]-self.joint_state[5])/50
                    self.joint_state[6]+=(self.res[1]-self.joint_state[6])/50
                    self.joint_state[7]+=(self.res[2]-self.joint_state[7])/50
                    print(f"{theta}  {box_theta}")
                    self.joint_state[8]=theta+box_theta
                else:
                    self.frame=t5-1  
            elif(self.frame==t1):
                self.delta_theta=(self.res-self.joint_state[5:8])/(t11-t1)
            elif(t1<self.frame<t11):
                self.joint_state[5:8]+=self.delta_theta
            elif(self.frame==t11):
                self.joint_state[5]=self.res[0]
                self.joint_state[6]=self.res[1]
                self.joint_state[7]=self.res[2]
                # print(f"here is {self.joint_state[11:13]}")
                self.uuid=uuid.uuid1().__str__()[:23]
            elif(t1<self.frame<t2):
                self.joint_state[11]-=np.pi/150
                self.joint_state[12]-=np.pi/150
            elif(self.frame==t2):
                
                # self.aim_dx=box_pose.p[0]-origin_pos[0]-0.03
                # self.aim_dz=box_pose.p[2]-origin_pos[2]+delta_h
                # self.aim_dy=box_pose.p[1]-(origin_pos[1]-0.0234)
                self.aim=np.array([-np.pi/6,np.pi/4,np.pi/4])
                self.delta_theta=self.aim-self.joint_state[5:8]
            elif(t2<self.frame<t3):
                # self.aim_dz+=0.0005
                # res=calculate(sqrt(self.aim_dx**2+self.aim_dy**2),self.aim_dz,np.pi*1.5) #抓取位置
                # if(res!=1):
                #     self.joint_state[5]=-res[0]
                #     self.joint_state[6]=res[1]
                #     self.joint_state[7]=res[2]
                
                self.joint_state[5:8]+=self.delta_theta/(t3-t2)
            # elif(self.frame==t4):
            #     self.joint_state=np.zeros(15)
            #     self.joint_state[11]=np.pi/3
            #     self.joint_state[12]=np.pi/3
            elif(self.frame==t3):
                print("start")
                start_pos=np.array(self.dc.get_rigid_body_pose(self.block).p)+np.random.rand(3)*0.1
                # start_pos[0]+=delta_x
                # start_pos[1]+=delta_y
                start_ori_euler=trans_euler(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block).r))
                print(f"{start_pos}  {start_ori_euler}")
                # print(f"{self.block_prim.get_default_state().position}  {quat_to_euler_angles(self.block_prim.get_default_state().orientation)}")
                rand_theta=(np.random.random()-0.5)*(np.pi/2)
                rand_ori=euler_angles_to_quat(np.array([0,0,rand_theta]))
                rand_x=0.32
                
                new_location = np.array([rand_x+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.0])  

                rand_theta=(np.random.random()-0.5)*(np.pi/12)
                
                rand_ori=euler_angles_to_quat(np.array([rand_theta,0,0]))
                
                # Create a new transform
                new_transform = _dynamic_control.Transform(new_location,rand_ori)
                self.dc.set_rigid_body_linear_velocity(self.block2, np.zeros(3))
                self.dc.set_rigid_body_angular_velocity(self.block2, np.zeros(3))
                self.dc.set_rigid_body_pose(self.block2, new_transform)
                # Apply the new transform
                box2_pose=self.dc.get_rigid_body_pose(self.block2)
                # print(box_pose.r)
                end_x=box2_pose.p[0]-delta_x
                end_y=box2_pose.p[1] # 原点的偏移
                end_z=box2_pose.p[2]+delta_h
                d=0.067
                y_d=-0.02
                # self.block2_dx=new_location[0]-d*np.cos(rand_theta)-origin_pos[0]-delta_x2
                # self.block2_dy=new_location[1]-d*np.sin(rand_theta)-origin_pos[1]+delta_y2
                end_pos=np.array([end_x-d*np.cos(rand_theta),end_y-d*np.sin(rand_theta)+y_d,end_z])
                self.end_pos=end_pos
                end_d1=0.16
                end_h=0.1
                end_pos2=np.array([end_x-end_d1*np.cos(rand_theta),end_y-end_d1*np.sin(rand_theta)+y_d,end_z+end_h])
                end_ori_euler=trans_euler(rand_ori)
                self.block2_theta=rand_theta
                # 移动一下
                # new_location = np.array([0.8+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.0])  
                # new_transform = _dynamic_control.Transform(new_location,rand_ori)
                
                self.init_target_pos=np.array(self.dc.get_rigid_body_pose(self.block2).p)
                self.init_target_ori=np.array(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block2).r))
                
                start_d=0.07
                end_d=0.07
                
                start_control_p=start_pos-start_d*start_ori_euler
                end_down_euler=np.cross(np.cross(end_ori_euler,np.array([0,0,-1])),end_ori_euler)
                end_control_p=end_pos-end_d*end_down_euler
            
                self.start_num=30
                p_num=place_time
                de_cas_num=p_num-self.start_num
                self.T=np.linspace(0,1,de_cas_num)
                self.path=np.zeros((de_cas_num,3)) # 贝塞尔路径
                self.result_list=np.zeros((p_num,3))
                self.result_list[0,0]=-self.joint_state[5]
                self.result_list[0,1]=self.joint_state[6]
                self.result_list[0,2]=self.joint_state[7]

                self.cur_theta=np.sum(self.joint_state[6:8])-self.joint_state[5]+np.pi/2
                self.d_theta=np.pi*1.5-self.cur_theta
                self.version =2
                if self.version==1:
                    for i in range(de_cas_num):
                        self.path[i]=de_casteljau(self.T[i],[start_pos,end_pos2,end_control_p,end_pos])
                        cur_theta=self.cur_theta+self.T[i]*self.d_theta
                        # print(cur_theta)
                        dx=self.path[i][0]-origin_pos[0]
                        dy=(self.path[i][1]-origin_pos[1])
                        dz=self.path[i][2]-origin_pos[2]
                        print(f"{sqrt(dx**2+dy**2)}, {dz}")
                        res=calculate(sqrt(dx**2+dy**2),dz,cur_theta)
                        if(res!=1):
                            self.result_list[self.start_num+i]=res
                    print(self.result_list)
                    self.result_list=linear_interpolation(self.result_list)
                
                if self.version==2:
                    DH[:,0]=self.result_list[0]
                    self.path2=np.zeros((p_num,3))
                    start_pos0=forward_kinematic(self.joint_state[0],DH)+origin_pos
                    T=np.linspace(0,1,p_num)
                    for i in range(p_num):
                        self.path2[i]=de_casteljau(T[i],[start_pos0,end_pos2,end_control_p,end_pos])-origin_pos
                    result_list=inverse_kinematic(self.path2,self.cur_theta,self.d_theta/p_num)
                    end_effector_pos=[]
                    for index,data in enumerate(result_list):
                        x=self.path2[index][0]
                        y=self.path2[index][1]
                        dphi=np.arctan(y/x)
                        DH[:,0]=data
                        cur_end_pos=forward_kinematic(dphi,DH)
                        end_effector_pos.append(cur_end_pos)

                    end_effector_pos=np.array(end_effector_pos)
                    end_effector_pos_new=np.zeros_like(end_effector_pos)

                    for i in range(3):
                        end_effector_pos_new[:,i] = savgol_filter(end_effector_pos[:,i], 50, 2, mode= 'nearest')
                    self.result_list=inverse_kinematic(end_effector_pos_new,self.cur_theta,self.d_theta/p_num)
                    print(self.result_list)
                # with open("F:/robot/data.csv",mode="a",newline='') as f:
                #     writer = csv.writer(f)
                #     cur_euler2=trans_euler(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block2).r))
                #     writer.writerow([self.uuid,0,0,0,0,0,0,end_pos[0],end_pos[1],end_pos[2],cur_euler2[0],cur_euler2[1],cur_euler2[2],0,0,0,0,0,0])
            elif(t3<self.frame<t4):

                # 未采取轨迹规划
                # dx=self.block2_dx
                # dy=-self.block2_dy # 原点的偏移
                # theta=-atan(dy/dx)
                # dz=-delta_z
                # self.res=calculate(sqrt(dx**2+dy**2),dz,np.pi*1.5)
                # print(f'ori dx {dx} dy {dy} dz {dz} ans {self.res}')
                # if(self.res!=1):
                #     self.joint_state[0]+=(theta-self.joint_state[0])/100
                #     self.joint_state[5]+=(-self.res[0]-self.joint_state[5])/100
                #     self.joint_state[6]+=(self.res[1]-self.joint_state[6])/100
                #     self.joint_state[7]+=(self.res[2]-self.joint_state[7])/100
                #     self.joint_state[8]+=(theta+self.block2_theta-self.joint_state[8])/100
                # else:
                #     self.frame=t5-1
                
                # 采取轨迹规划
                
                
                    # cur_theta=np.sum(self.joint_state[6:8])-self.joint_state[5]+np.pi/2
                    # for ang in np.linspace(np.pi/4,90):
                    #     cur_ans=calculate(sqrt(dx**2+dy**2),dz,cur_theta+ang)
                    #     if cur_ans!=1:
                    #         self.res=cur_ans
                    #         print(f'dx {dx} dy {dy} dz {dz} theta {cur_theta+ang} ans {self.res}')
                    #         break
                    #     self.res=calculate(sqrt(dx**2+dy**2),dz,cur_theta-ang)
                    #     if cur_ans!=1:
                    #         self.res=cur_ans
                    #         print(f'dx {dx} dy {dy} dz {dz} theta {cur_theta-ang} ans {self.res}')
                    #         break
                i=self.frame-t3-1
                if self.version==1:
                    dx=self.path[i-self.start_num][0]-origin_pos[0]
                    dy=(self.path[i-self.start_num][1]-origin_pos[1])
                    theta=-atan(dy/dx)
                elif self.version==2:
                    dx=self.path2[i][0]
                    dy=(self.path2[i][1])
                    theta=-atan(dy/dx)
                
                self.joint_state[0]+=(theta-self.joint_state[0])/100
                self.joint_state[5]=-self.result_list[i][0]
                self.joint_state[6]=self.result_list[i][1]
                self.joint_state[7]=self.result_list[i][2]
                self.joint_state[8]+=(theta+self.block2_theta-self.joint_state[8])/100
                # else:
                #     self.frame=t5-1
                if(self.frame%10==0):
                    # self.camera.get_current_frame()["motion_vectors"]
                    # print(self.camera.get_current_frame())
                    image = Image.fromarray(self.camera.get_current_frame()['rgba'] )
                    # Save the image to a file
                    index=i//10
                    # image.save(f'F:/robot/output/{self.uuid}-{index}.png')
                    pos=self.dc.get_rigid_body_pose(self.block).p
                    cur_euler=trans_euler(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block).r))
                    pos2=self.dc.get_rigid_body_pose(self.block2).p
                    cur_euler2=trans_euler(quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block2).r))
                    # with open("F:/robot/data.csv",mode="a",newline='') as f:
                    #     writer = csv.writer(f)
                    #     writer.writerow([self.uuid,index,self.joint_state[0],self.joint_state[5],self.joint_state[6],self.joint_state[7],self.joint_state[8],pos[0],pos[1],pos[2],cur_euler[0],cur_euler[1],cur_euler[2],pos2[0],pos2[1],pos2[2],cur_euler2[0],cur_euler2[1],cur_euler2[2]])
            elif(self.frame==t4):
                print("end")
                plt.plot(self.reward_list)
                plt.xlabel('Episode')
                plt.ylabel('Reward')
                plt.title('Reward per Episode')
                plt.grid(True)
                plt.savefig("F:/robot/graph/reward.png")
                plt.close()

            
                print(f"{self.dc.get_rigid_body_pose(self.block)}  {quat_to_euler_angles(self.dc.get_rigid_body_pose(self.block).r)}")
                # print(f"{self.block_prim.get_default_state().position}  {quat_to_euler_angles(self.block_prim.get_default_state().orientation)}")
            elif(t4<self.frame<t5):
                self.joint_state[11]+=np.pi/150
                self.joint_state[12]+=np.pi/150
            elif(self.frame==t5):
                rand_x=0.35
                new_location = np.array([rand_x+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.00])  

                rand_theta=(np.random.random()-0.5)*(np.pi/2)
                
                rand_ori=euler_angles_to_quat(np.array([rand_theta,0,0]))
                
                # Create a new transform
                # new_transform = _dynamic_control.Transform(new_location,rand_ori)
                # self.dc.set_rigid_body_linear_velocity(self.box, np.zeros(3))
                # self.dc.set_rigid_body_angular_velocity(self.box, np.zeros(3))
                # # Apply the new transform
                # self.dc.set_rigid_body_pose(self.box, new_transform)
                self.aim=np.array([-np.pi/6,np.pi/4,np.pi/4])
                self.aim2=np.array([0.52359878 ,0.52359878])
                self.delta_theta=self.aim-self.joint_state[5:8]
                self.delta_theta2=self.aim2-self.joint_state[11:13]
            elif(t5<self.frame<t6):
                self.joint_state[5:8]+=self.delta_theta/(t6-t5)
                self.joint_state[11:13]+=self.delta_theta2/(t6-t5)
            elif(self.frame==t6):
                rand_x=0.22
                new_location = np.array([rand_x+rand_pos_range*np.random.random(), rand_pos_range*np.random.random(), 0.0])  

                rand_theta=(np.random.random()-0.5)*(np.pi/3)
                
                rand_ori=euler_angles_to_quat(np.array([rand_theta,0,0]))
                
                # Create a new transform
                new_transform = _dynamic_control.Transform(new_location,rand_ori)
                # Apply the new transform
                self.dc.set_rigid_body_pose(self.block, new_transform)
                self.frame=0
        elif(mode==3):
            t1=70
            t11=100
            t2=150
            t3=250
            place_time=300
            t4=t3+place_time
            t5=t4+100
            t6=t5+100
            t7=700
            
            if(self.frame==t1):
                self.aim=np.array([-np.pi/6,np.pi/4,np.pi/4])
                self.delta_theta=self.aim-self.joint_state[5:8]
            elif(t1<self.frame<t2):
                self.joint_state[5:8]+=self.delta_theta/(t3-t2)
            elif(self.frame==t2):
                joints=self.joint_state[5:8]
                print("joint",joints)
                joints[0]=-joints[0]
                DH = np.array([
                    [joints[0],0.10513],
                    [joints[1],0.09761],
                    [joints[2],0.17093],
                    ])
                p_list,theta1=get_all_state(DH)
                self.start_pos=p_list[-1] # 最初点
                self.start_pos[-1]=theta1
                self.start_pos[0]=-self.start_pos[0]
                self.cur_pos=copy.deepcopy(self.start_pos)
            elif(self.frame>t2):
                try:
                    cur_data=read_headers(file_path)
                    print(cur_data)
                    dx=cur_data["x"]/factor
                    dy=cur_data["y"]/factor
                    # print("dx",dx,"dy",dy)
                    dtheta=radians(cur_data["Angle"])/3
                    is_reset=cur_data["Reset"]
                    if is_reset==1:
                        with open(file_path, mode='r', newline='') as file:
                            reader = list(csv.reader(file))
                    
                        # 创建新的第二行
                        new_row = [cur_data.get("x", ""), cur_data.get("y", ""), cur_data.get("Angle", ""), cur_data.get("Angle2", ""), 0.0]
                        
                        # 更新第二行
                        reader[1] = new_row
                        
                        # 写回CSV文件
                        with open(file_path, mode='w', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerows(reader)
                        self.start_pos=copy.deepcopy(self.cur_pos)
                        # start_pos[0]=-start_pos[0]
                    cur_x=self.start_pos[0]+dx
                    cur_y=self.start_pos[1]+dy
                    # print("cur_y",start_pos[1])
                    cur_theta=self.start_pos[2]+dtheta
                    cur_ans=calculate(cur_x,cur_y,cur_theta)
                    # print("cur_input",cur_x,cur_y,cur_theta)
                    # print("cur_ans",cur_ans)
                    print(cur_ans)
                    if(cur_ans!=1):
                        dphi=radians(cur_data["Angle2"])
                        self.joint_state[0]=dphi
                        cur_ans[0]=-cur_ans[0]
                        self.joint_state[5:8]=cur_ans
                        # print(p_list_tmp)
                        self.cur_pos[0]=cur_x
                        self.cur_pos[1]=cur_y
                        self.cur_pos[2]=cur_theta
                except:
                    print("fail to read")     

        robot_view.apply_action(ArticulationAction(joint_positions=self.joint_state))
        return

    def world_cleanup(self):
        return
