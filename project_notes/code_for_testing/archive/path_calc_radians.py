# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_calc_radians.py
import math
from sympy import Eq, symbols, solve

path_data = [
"LINESTRING (-6.373268939109791 -3.8884440636760638, -8.914302735073203 3.4912539271027665)",
"LINESTRING (-5.195466293158876 -4.837644520732015, -11.07529003323793 12.238603544022125)",
"LINESTRING (-4.011300363905062 -5.805325294373941, -10.84625596421566 14.044827108479351)",
"LINESTRING (-2.8596959749679516 -6.678440488434236, -10.470395137867168 15.424634807175941)",
"LINESTRING (-1.5588777207480118 -7.984904213149015, -10.55621925331021 18.1452729358826)"
]

# Define the coordinates of the first point (start)
x1, y1 = -6.373268939109791, -3.8884440636760638 # Line 1, first point  - go up starting at this point at a heading of 1.902
x2, y2 = -8.914302735073203, 3.4912539271027665  # second point - end of line 1
angle_of_travel = math.atan2(y2 - y1, x2 - x1)
print("LINESTRING 1 The angle of travel in radians is:", angle_of_travel)

# what is the intersecting point of the point x2, y2 and the line segment (x3, y3), (x4, y4)
x3, y3 = -5.195466293158876, -4.837644520732015  # Line 2, second point - end of line 2
x4, y4 = -11.07529003323793, 12.238603544022125 
angle_of_travel = math.atan2(y4 - y3, x4 - x3)
print("LINESTRING 2 The angle of travel in radians is:", angle_of_travel)
# what is the intersecting point of the point x2, y2 and the line segment (x3, y3), (x4, y4)

m1 = (y2 - y1) / (x2 - x1)		# Calculate the slope of the line segment (x1, y1) to (x2, y2)
m2 = -1 / m1					# Calculate the slope of the perpendicular line
x, y = symbols('x y')
eq_perpendicular = Eq((y - y2), m2 * (x - x2))  # Equation of the perpendicular line through (x2, y2)
# Equation of the line segment (x3, y3) to (x4, y4)
m_segment = (y4 - y3) / (x4 - x3)
eq_line_segment = Eq((y - y3), m_segment * (x - x3))
# Solve the system of equations to get the intersection point
intersection = solve((eq_perpendicular, eq_line_segment), (x, y))
print("First intersection point:", intersection)
# Extract intersection point
intersection_point1 = (intersection[x], intersection[y])  # Line 2, first point - start at this point and go down at a heading of 5.042

# after line 2, second point (point 4) we need to calculate the next intersecting point Line 3, point 1 (point 5)
# what is the intersection point?  What data do I need?



x5, y5 = -4.011300363905062, -5.805325294373941, 
x6, y6 = -10.84625596421566, 14.044827108479351
angle_of_travel = math.atan2(y6 - y5, x6 - x5)
print("LINESTRING 3 The angle of travel in radians is:", angle_of_travel)





x2, y2 = -8.914302735073203, 3.4912539271027665 
x3, y3 = -5.195466293158876, -4.837644520732015  
x4, y4 = -11.07529003323793, 12.238603544022125 