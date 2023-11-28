# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/path_test_circle_calc.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Define the dataframe with the field segments
field_segments = pd.DataFrame({
    'x1': [ -6.373,  -3.888],
    'y1': [ -8.914,   3.491],
    'x2': [ -5.195,  -4.837],
    'y2': [-11.075,  12.238]
})

# LINESTRING (-6.373268939109791 -3.8884440636760638, -8.914302735073203 3.4912539271027665)
# LINESTRING (-5.195466293158876 -4.837644520732015, -11.07529003323793 12.238603544022125)


# Calculate the direction vectors for both line segments and normalize them
dir_vec1 = np.array([field_segments.loc[0, 'x2'] - field_segments.loc[0, 'x1'],
                     field_segments.loc[0, 'y2'] - field_segments.loc[0, 'y1']])
dir_vec1 /= np.linalg.norm(dir_vec1)

# Determine the slope of the yellow line (which is parallel to the red line)
slope_yellow = dir_vec1[1] / dir_vec1[0]

# Calculate the y-intercept of the yellow line using one of its points
yellow_intercept = (field_segments.loc[0, 'y2'] + field_segments.loc[1, 'y2']) / 2 - slope_yellow * (field_segments.loc[0, 'x2'] + field_segments.loc[1, 'x2']) / 2

# Calculate the center of the circle to be tangent to the red line at its top point
tangent_point_red = np.array([field_segments.loc[0, 'x2'], field_segments.loc[0, 'y2']])
perpendicular_slope = -1 / slope_yellow
perpendicular_intercept = tangent_point_red[1] - perpendicular_slope * tangent_point_red[0]

# Calculate the intersection point of the yellow line and the line perpendicular to the red line
circle_center_x = (yellow_intercept - perpendicular_intercept) / (perpendicular_slope - slope_yellow)
circle_center_y = slope_yellow * circle_center_x + yellow_intercept
circle_center = np.array([circle_center_x, circle_center_y])

# Calculate the radius based on the circle's center and the tangent point on the red line
radius = np.linalg.norm(circle_center - tangent_point_red)

# Calculate the endpoints for the yellow line
yellow_line_length = np.hypot(field_segments.loc[0, 'x2'] - field_segments.loc[0, 'x1'], field_segments.loc[0, 'y2'] - field_segments.loc[0, 'y1'])
yellow_x1 = circle_center_x - dir_vec1[0] * yellow_line_length / 2
yellow_y1 = circle_center_y - dir_vec1[1] * yellow_line_length / 2
yellow_x2 = circle_center_x + dir_vec1[0] * yellow_line_length / 2
yellow_y2 = circle_center_y + dir_vec1[1] * yellow_line_length / 2

# Plot the line segments and the circle
plt.figure(figsize=(10, 6))
plt.plot([field_segments.loc[0, 'x1'], field_segments.loc[0, 'x2']],
         [field_segments.loc[0, 'y1'], field_segments.loc[0, 'y2']], 'r-', label='First Line Segment')
plt.plot([field_segments.loc[1, 'x1'], field_segments.loc[1, 'x2']],
         [field_segments.loc[1, 'y1'], field_segments.loc[1, 'y2']], 'b-', label='Second Line Segment')
plt.plot([yellow_x1, yellow_x2], [yellow_y1, yellow_y2], 'y-', lw=2, label='Yellow Line')

# Draw the circle on the plot
circle = plt.Circle(circle_center, radius, color='green', fill=False, linestyle='--', label='Turning Circle')
plt.gca().add_patch(circle)

# Mark the circle center
plt.plot(circle_center[0], circle_center[1], 'go', label='Circle Center')

# Mark the tangent point on the red line
plt.plot(tangent_point_red[0], tangent_point_red[1], 'rx', label='Tangent Point on Red Line')

# Set plot details to make x and y axis equal
plt.axis('equal')
plt.grid(True)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Circle Tangent to the Top of the Red Line and Centered on the Yellow Line')
plt.legend()
plt.show()
