#!/usr/bin/env python3
'''
print_test.py
'''
s = "Price: $ %6.2f"% (356.08977)
print(s)
print("Second argument: {1:3d}, first one: {0:7.2f}".format(47.42,11))
y = "label 1 {1:3f}, label 2: {0:7.2f}".format(12345.67890,98765.4321)
print(y)
x=17.179181081989352
y=3.8921906054196236
yaw=-0.11881537452946524
q_x=4.110019047626808e-07
q_y=2.2266633280817393e-06
q_z=-0.059372749103616285
q_w=0.9982358822736999

value_string = "{0:4.2f}, {1:4.2f}, {2:4.3f}, {3:4.4f}, {4:4.4f}, {5:4.4f}, {6:4.4f}".format(x, y, yaw, q_x, q_y, q_z, q_w)
print(value_string)
