import kinematic
import numpy as np


origin_pos = np.array([0.05857364, -0.00254144,  0.1743498])
start_pos = np.array([0.02787045 ,0.00068883, 0.4154592]) - origin_pos
end_pos = np.array([0.24 ,-3.17631695e-05 , 9.82053450e-02 ]) - origin_pos

path_list=kinematic.generate_path(start_pos,end_pos,end_d=0.03)
result_list=kinematic.select_alpha(path_list,np.pi/2,-1.2-0.5-1.5+np.pi/2)
angle_list=kinematic.inverse_kinematic_with_alpha(path_list,result_list)