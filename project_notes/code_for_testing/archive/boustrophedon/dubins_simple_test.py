#!/usr/bin/env python3
'''
python3 dubins_simple_test.py
cd /home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing

testing dubins calculation
'''

import dubins
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
import math

#21.895 ,12.49
# 20.61 ,13.71

x0=21.895
y0=12.49
x1=20.61
y1=13.71
theta0=1.67
theta1=-1.67

q0 = (x0, y0, theta0)
q1 = (x1, y1, theta1)
turning_radius = 2.6
step_size = 0.8

path = dubins.shortest_path(q0, q1, turning_radius)
#print(type(path))
configurations, _ = path.sample_many(step_size)
#print(type(path.sample_many(step_size)))
#print(path.sample_many(step_size))
#print('length=',len(path.sample_many(step_size)));
dub_tuple = path.sample_many(step_size)
#print('first part:', dub_tuple[0])
#print('length=',len(dub_tuple[0]));
#print('second part:', dub_tuple[1])
#print('length=',len(dub_tuple[1]));
#print('starting x, y, yaw:', x0, y0, theta0)
px = []
py = []
pyaw = []
#px.append(round(x0,2))
#py.append(round(y0,2))
for x in dub_tuple[0]:
	#print("x, y, yaw", round(x[0],2), round(x[1],2), round(x[2],2))
	px.append(round(x[0],2))
	py.append(round(x[1],2))
	if (x[2]< math.pi):
		pyaw.append(round(x[2],2))
	else:  # the dubins calc use 2*Pi() for 360 rotation instead of -pi to +pi
		tyaw = (math.pi-(x[2]-math.pi))*-1
		pyaw.append(round(tyaw,2))
#print('ending x, y, yaw:', x1, y1, theta1)
px.append(round(x1,2))
py.append(round(y1,2))
pyaw.append(round(theta1,2))
# print statements for mission script
print(px)
print(py)
print(pyaw)

# print the 'create_pose' statements with x, y, z, quaternion(4) format
# find the enumerate example
# for i, line in enumerate(lines_f1, start = 0):
for i, coord in enumerate(px):
    x = coord
    y = py[i]
    yaw = pyaw[i]
    quat = quaternion_from_euler(0, 0, yaw)
    print("create_pose(", round(x,2), "," , round(y,2), ",", 0, ",", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4), "),") 


# plot output
plt.legend()
plt.grid(True)
plt.plot(px, py)
plt.title("Cutting Hay Path")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()