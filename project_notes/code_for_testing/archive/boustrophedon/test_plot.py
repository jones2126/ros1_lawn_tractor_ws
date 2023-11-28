import matplotlib.pyplot as plt
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/test_plot.py

# Data points
data = [
    [-5.0, -12.0, 5.924353243901755],
    [-2.86, -6.678, 1.902381653857587],
    [-10.47, 15.425, 0.3319783645422508],
    [-9.71, 15.687, 5.044004436383807],
    [-1.559, -7.985, 0.3315734211637279],
    [-0.798, -7.723, 1.9024248677612476],
    [-11.315, 22.819, 0.3315734211637282],
    [-10.554, 23.081, 5.044009529517675],
    [2.141, -13.787, 5.044009529517675]
]

plt.figure(figsize=(8, 8))

# Plotting line segments and points with sequence numbers
for i, point in enumerate(data):
    plt.scatter(point[0], point[1], color='blue')  # Plot point
    plt.text(point[0], point[1], f' {i+1}', verticalalignment='bottom', horizontalalignment='right')  # Sequence number

    # Draw line segment to next point if not the last point
    if i < len(data) - 1:
        next_point = data[i+1]
        plt.plot([point[0], next_point[0]], [point[1], next_point[1]], 'r-')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Connected Line Segments with Sequence Numbers')
plt.grid(True)
plt.axis('equal')  # Ensure equal aspect ratio
plt.show()

