# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_plot_single_dubins_shortest.py
import matplotlib.pyplot as plt
import numpy as np
import dubins

def generate_path(x0, y0, x1, y1, theta0, theta1, turning_radius, step_size):
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    continuous_path = dubins.shortest_path(q0, q1, turning_radius)
    calculated_path_segment, _ = continuous_path.sample_many(step_size) # takes the continuous Dubins path and samples it at intervals specified by step_size
    return calculated_path_segment

# Example usage of the function
x0, y0, theta0 = 20.0, 0.2, 0
x1, y1, theta1 = 20.0, 0.7, 3.14 
turning_radius=1.3
step_size=0.3

calculated_path_segment = generate_path(x0, y0, x1, y1, theta0, theta1, turning_radius, step_size)

# Extracting X, Y, and Theta values
x_values = [point[0] for point in calculated_path_segment]
y_values = [point[1] for point in calculated_path_segment]
theta_values = [point[2] for point in calculated_path_segment]

# Plotting the path
plt.figure(figsize=(10, 6))
plt.plot(x_values, y_values, 'o-', label='Path')
for i in range(0, len(calculated_path_segment), 5):  # Plotting every 5th point with an arrow
    plt.arrow(x_values[i], y_values[i], np.cos(theta_values[i])*0.1, np.sin(theta_values[i])*0.1, head_width=0.05, head_length=0.1, fc='blue', ec='blue')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Generated Dubins Path')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()
