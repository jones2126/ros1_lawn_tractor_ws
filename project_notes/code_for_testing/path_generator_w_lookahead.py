
#!/usr/bin/env python3
# python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_generator_w_lookahead.py
import dubins
import math
import matplotlib.pyplot as plt

# Variables
show_animation = True
x0 = 0.0
y0 = 0.0
theta0 = 0.0
x1 = 0.0
y1 = 0.0
theta1 = 0.0
speed = 0.0
drive_path = []
drive_points = []

start_x = 0.0
start_y = 0.0
start_yaw = 0.0
total_count = 0 
turning_radius = 1.3    
step_size = 0.1         

# Added lines
straight_line_threshold = 0.05  
straight_line_lookahead = 2.5
curve_lookahead = 1.5

def generate_path(x0, y0, x1, y1, theta0, theta1):
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)
    return configurations
    return []

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# def get_lookahead(theta0, theta1):
#     heading_diff = abs(theta1 - theta0)
#     if heading_diff < straight_line_threshold:
#         return straight_line_lookahead
#     else:
#         return curve_lookahead

# main routine
#input_file_waypoints = "input_file.txt"
input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_uturn_input4.txt"
output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt"
print("Output file:", output_file_waypoints)

with open(input_file_waypoints, 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]

    for index, line in enumerate(content):
        #print(f"Processing Input Line {index + 1}: {line}")  # This will print the line number and the content of the line
        points = line.split()
        if line == content[0]:
            # Start of file just load waypoint
            # then get next line
            x1 = float(points[0])
            y1 = float(points[1])
            theta1 = float(points[2])
            lookahead = float(points[3])            
            speed = float(points[4])
            print(f"Record {index + 1}: x1 = {x1}, y1 = {y1}, theta1 = {theta1}")  # Print the values of x1, y1, and theta1
        else:
            #if line == content[len(content)-1]:
            print('end of file')
            x1 = float(points[0])
            y1 = float(points[1])
            theta1 = float(points[2])
            lookahead = float(points[3])
            speed = float(points[4])
            path = generate_path(x0,y0,x1,y1,theta0,theta1)
            print(f"Record {index + 1}: x1 = {x1}, y1 = {y1}, theta1 = {theta1}")  # Print the values of x1, y1, and theta1
            for i in path:
                i = i + (lookahead, speed)
                drive_path.append(i)

        # The final waypoint becomes the start of the
        # next segment.
        x0 = x1
        y0 = y1
        theta0 = theta1
        #speed = speed


    if show_animation:
        px = []
        py = []
        pyaw = []
        with open(output_file_waypoints, 'w') as file:
            count = 0
            for p in drive_path:
                if count > 20:
                    px.append(p[0])
                    py.append(p[1])
                    pyaw.append(p[2])
                    count = 0
                count += 1
                total_count += 1 
                drive_points.append([p[0],p[1]])
                file.write(str(p[0]) + " " + str(p[1]) + " " + str(p[2]) + " " + str(p[3]) + " " + str(p[4]) +"\n")
        plt.plot(0,0, label="final course")
        plt.plot(*zip(*drive_points))

        for (ix, iy, iyaw) in zip(px, py, pyaw):
            plot_arrow(ix, iy, iyaw, fc="b")

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()
