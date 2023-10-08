#!/usr/bin/env python
'''
Script that helps build a path to follow using four corner inputs

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_first_planning_helper.py

'''
# Define the coordinates of the lower left and lower right corner points


# Upper left: 12.9, 16.3
# Upper right: 23.3, 18.9
# Lower left: 15.9, 0
# Lower right: 22.3, 0.0
upper_left_x = 12.9
upper_right_x = 23.3
lower_left_x = 15.9
lower_right_x = 22.3

upper_left_y = 16.3
upper_right_y = 18.9
lower_left_y = 0.0
lower_right_y = -0.4

# Calculate the middle x-coordinate
middle_x_lower = (lower_left_x + lower_right_x) / 2
middle_x_upper = (upper_left_x + upper_right_x) / 2
middle_y_lower = (lower_left_y + lower_right_y) / 2
middle_y_upper = (upper_left_y + upper_right_y) / 2

# Print the result
print("The middle coordinate on the lower part of the polygon is:", middle_x_lower, middle_y_lower)
print("The middle coordinate on the upper part of the polygon is:", middle_x_upper, middle_y_upper)
