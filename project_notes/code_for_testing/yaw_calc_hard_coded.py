#! /usr/bin/python3
# input a yaw and output a radian
# different angles based on increments of pi()
import math 
from tf.transformations import quaternion_from_euler

# main routine
yaw = 0
circle_radians = math.pi*2

increment = math.pi/4
print("Increment:", round(increment, 3))
while yaw <= circle_radians:
    quat = quaternion_from_euler(0, 0, yaw)
    print("yaw:", round(yaw,2), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))
    yaw += increment
else:
    quat = quaternion_from_euler(0, 0, math.pi*2)
    print("final yaw:", round(yaw,3), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))   

yaw = 0    
increment = math.pi/6
print("Increment:", round(increment, 3))
while yaw <= circle_radians:
    quat = quaternion_from_euler(0, 0, yaw)
    print("yaw:", round(yaw,2), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))
    yaw += increment
else:
    quat = quaternion_from_euler(0, 0, math.pi*2)
    print("final yaw:", round(yaw,3), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))   


yaw = 0    
increment = math.pi/8
print("Increment:", round(increment, 3))
while yaw <= circle_radians:
    quat = quaternion_from_euler(0, 0, yaw)
    print("yaw:", round(yaw,2), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))
    yaw += increment
else:
    quat = quaternion_from_euler(0, 0, math.pi*2)
    print("final yaw:", round(yaw,3), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))   

yaws = [1.8623, 2.3562, 2.8501, -2.8501, -2.3562, -1.8623, -1.2793, -0.7854, -0.2915, 0.2915, 0.7854, 1.2793]
for yaw in yaws:
    quat = quaternion_from_euler(0, 0, yaw)
    print("final yaw:", round(yaw,4), ",quat:", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4))   
