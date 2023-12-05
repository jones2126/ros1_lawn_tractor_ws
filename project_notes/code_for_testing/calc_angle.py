#!/usr/bin/env python3
'''
calc_angle.py

Find the Euclidean distance between one two dimensional points.

ref: https://stackoverflow.com/questions/37259366/using-python-to-calculate-radial-angle-in-clockwise-counterclockwise-directions

'''
from math import sin, cos, radians, pi, atan2, degrees

def angle_to(p1, p2, rotation=0, clockwise=False):
    angle = degrees(atan2(p2[1] - p1[1], p2[0] - p1[0])) - rotation
    if not clockwise:
        angle = -angle
    return angle % 360

def angle_to_rad(p1, p2, rotation=0, clockwise=False):
    angle = (atan2(p2[1] - p1[1], p2[0] - p1[0])) - rotation
    if not clockwise:
        angle = -angle
    return angle     

# main routine
point1 = [2,0]
point2 = [1.7,1.0]
print(angle_to(point1, point2, 0, True))
print(angle_to_rad(point1, point2, 0, True))
print(angle_to(point1, point2, 0, False))
print(angle_to_rad(point1, point2, 0, False))

point1 = [1.7,1.0]
point2 = [1.0,1.7]
print(angle_to(point1, point2, 0, True))
print(angle_to_rad(point1, point2, 0, True))
print(angle_to(point1, point2, 0, False))
print(angle_to_rad(point1, point2, 0, False))