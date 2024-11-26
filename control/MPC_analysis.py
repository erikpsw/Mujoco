# main.py

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from MPC_solver import simulate_mpc, plot_trajectory
import dynamic_model_linear as dml
import time
# Simulation parameters
k_steps = 2000
N = 50
t = 0.05

num_control = 4
control_points = [0.2, 0.3, 0.5, 0.9]
initial_state = np.array([-0.1, 0.1, np.pi/2])
num_sample=50

# Run the simulation
X_k, U_k, X, path, pred_list = simulate_mpc(k_steps, N, t, num_control, control_points, initial_state, num_samples=num_sample)
# Create a figure and axis
fig, ax = plt.subplots()

# Draw the robot's body outline
car_model = dml.DifferentialDriveRobot()
car_model.state = initial_state
wheel_positions = car_model.get_wheel_positions_world()
hull = plt.Polygon(wheel_positions, fill=False, color='blue')
ax.add_patch(hull)

 # Plot the future N steps
future_path, = ax.plot(pred_list[0][:, 0], pred_list[0][:, 1], 'r--')

sample_idx = 0
def animate(i):
    global num_sample, sample_idx
    
    if sample_idx<num_sample-1:
        # print("Sample: ", sample_idx)
        
        future_path.set_data(pred_list[sample_idx][:, 0], pred_list[sample_idx][:, 1])
        car_model.state = pred_list[sample_idx][0]
        wheel_positions = car_model.get_wheel_positions_world()
        hull.set_xy(wheel_positions)
        sample_idx += 1
    if sample_idx == 0:
        time.sleep(2)
    return [hull, future_path]

plot_trajectory(X_k, X, path, N)


ani = animation.FuncAnimation(fig, animate, interval=200, blit=False)

plt.show()