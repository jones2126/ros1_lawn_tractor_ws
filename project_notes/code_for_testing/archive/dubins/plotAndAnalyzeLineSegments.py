# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/plotAndAnalyzeLineSegments.py
import matplotlib.pyplot as plt

# Data for the line segments
line_segments = [
    (-6.373, -3.888, -8.914, 3.491),
    (-5.195, -4.838, -11.075, 12.239),
    (-4.011, -5.805, -10.846, 14.045),
    (-2.86, -6.678, -10.47, 15.425),
    (-1.559, -7.985, -10.556, 18.145),
    (0.136, -10.436, -11.315, 22.819)
]

# Plotting each line segment
plt.figure(figsize=(10, 6))
for x1, y1, x2, y2 in line_segments:
    plt.plot([x1, x2], [y1, y2], marker='o')

# Set the labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of Line Segments with Equal Axes')
plt.grid(True)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.axis('equal')  # This will make the axes scale the same
plt.show()
