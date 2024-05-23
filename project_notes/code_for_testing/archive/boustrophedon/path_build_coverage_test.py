# Import the coverage_path_planner function
from coverage_path_planner import coverage_path_planner

# Define the polygon vertices
line_segments = [(1.0, -25.3), (-30, -37.2), (-45, -3.4), (-12.3, 14.7)]

# Define parameters
tool_width = 3.5
# 3.5 feet (42" in feet)
separation = tool_width * 1.1
# 10% overlap
margin = tool_width * 0.1
# Margin from the polygon edges
# Generate coverage path using coverage_path_planner
path_points = coverage_path_planner(polygon, tool_width, separation, margin)
