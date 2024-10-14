import kinematic
import numpy as np

origin_pos = np.array([0.0291812, -0.00117808, 0.1737953])
start_pos = np.array([0.02787045, 0.00068883, 0.51154592]) - origin_pos
end_pos = np.array([0.28808175 ,-0.02692359 , 0.10363261 ]) - origin_pos

path_list=kinematic.generate_path(start_pos,end_pos,end_d=0.05)
result_list=kinematic.select_alpha(path_list,np.pi/2,1.2+0.5+1.5)