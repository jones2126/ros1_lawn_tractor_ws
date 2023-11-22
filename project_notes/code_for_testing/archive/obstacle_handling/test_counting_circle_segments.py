# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_counting_circle_segments.py
import matplotlib.pyplot as plt
import numpy as np

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

def calculate_variable_paths(segments, intersect_segment_1, intersect_segment_2):
    # Extracting the points from the segments
    points = [seg[0] for seg in segments] + [segments[-1][1]]

    # Adjust for zero-based indexing
    intersect_segment_1 -= 1
    intersect_segment_2 -= 1

    # Midpoints of the specified segments
    midpoint_1 = ((points[intersect_segment_1][0] + points[intersect_segment_1 + 1][0]) / 2, 
                  (points[intersect_segment_1][1] + points[intersect_segment_1 + 1][1]) / 2)
    midpoint_2 = ((points[intersect_segment_2][0] + points[intersect_segment_2 + 1][0]) / 2, 
                  (points[intersect_segment_2][1] + points[intersect_segment_2 + 1][1]) / 2)

    # Clockwise and counterclockwise points
    clockwise_points = [midpoint_1] + points[intersect_segment_1 + 1:intersect_segment_2 + 1] + [midpoint_2]
    
    if intersect_segment_1 < intersect_segment_2:
        counterclockwise_points = [midpoint_1] + points[intersect_segment_1::-1] + points[-1:intersect_segment_2:-1] + [midpoint_2]
    else:
        counterclockwise_points = [midpoint_1] + points[intersect_segment_1::-1] + points[:intersect_segment_2:-1] + [midpoint_2]

    return points, midpoint_1, midpoint_2, clockwise_points, counterclockwise_points

def plot_polygon_and_paths(points, midpoint_1, midpoint_2, clockwise_points, counterclockwise_points, intersect_segment_1, intersect_segment_2):
    plt.figure(figsize=(10, 8))
    x, y = zip(*points)
    
    # Plotting the polygon
    plt.plot(x, y, label="Polygon", color="grey")
    
    # Adjust for zero-based indexing for labeling
    intersect_segment_1_label = intersect_segment_1
    intersect_segment_2_label = intersect_segment_2

    # Highlighting the specified segments
    plt.plot([points[intersect_segment_1 - 1][0], points[intersect_segment_1][0]], 
             [points[intersect_segment_1 - 1][1], points[intersect_segment_1][1]], 
             label=f"{intersect_segment_1_label}th Segment", color="blue", linewidth=2)
    plt.plot([points[intersect_segment_2 - 1][0], points[intersect_segment_2][0]], 
             [points[intersect_segment_2 - 1][1], points[intersect_segment_2][1]], 
             label=f"{intersect_segment_2_label}th Segment", color="green", linewidth=2)
    
    # Drawing the paths
    plt.plot(*zip(*clockwise_points), label="Clockwise Path", color="red", linestyle="--")
    plt.plot(*zip(*counterclockwise_points), label="Counterclockwise Path", color="orange", linestyle=":")

    # Marking the midpoints
    plt.scatter(*midpoint_1, color="black", zorder=5)
    plt.scatter(*midpoint_2, color="black", zorder=5)
    plt.text(*midpoint_1, f'Midpoint {intersect_segment_1_label}', verticalalignment='bottom')
    plt.text(*midpoint_2, f'Midpoint {intersect_segment_2_label}', verticalalignment='bottom')

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Polygon with Variable Intersecting Segments")
    plt.title(f'Script: {script_name}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
first_segment = 2
second_segment = 12
segments = [((20.3, -9.5), (20.167852593996916, -8.665654115187643)), ((20.167852593996916, -8.665654115187643), (19.78434588481236, -7.912979818810323)), ((19.78434588481236, -7.912979818810323), (19.18702018118968, -7.315654115187641)), ((19.18702018118968, -7.315654115187641), (18.43434588481236, -6.932147406003086)), ((18.43434588481236, -6.932147406003086), (17.6, -6.8)), ((17.6, -6.8), (16.765654115187644, -6.932147406003085)), ((16.765654115187644, -6.932147406003085), (16.012979818810322, -7.315654115187641)), ((16.012979818810322, -7.315654115187641), (15.415654115187643, -7.912979818810323)), ((15.415654115187643, -7.912979818810323), (15.032147406003087, -8.665654115187643)), ((15.032147406003087, -8.665654115187643), (14.900000000000002, -9.5)), ((14.900000000000002, -9.5), (15.032147406003087, -10.334345884812357)), ((15.032147406003087, -10.334345884812357), (15.415654115187643, -11.087020181189677)), ((15.415654115187643, -11.087020181189677), (16.012979818810322, -11.684345884812359)), ((16.012979818810322, -11.684345884812359), (16.765654115187644, -12.067852593996914)), ((16.765654115187644, -12.067852593996914), (17.6, -12.2)), ((17.6, -12.2), (18.43434588481236, -12.067852593996914)), ((18.43434588481236, -12.067852593996914), (19.187020181189677, -11.684345884812359)), ((19.187020181189677, -11.684345884812359), (19.78434588481236, -11.08702018118968)), ((19.78434588481236, -11.08702018118968), (20.167852593996916, -10.33434588481236)), ((20.167852593996916, -10.33434588481236), (20.3, -9.5))]
points, midpoint_1, midpoint_2, clockwise_points, counterclockwise_points = calculate_variable_paths(segments, first_segment, second_segment)
# Verifying the segment count based on the user's description

# Rough estimate of which segment is shorter is based on counting the number of segments each path consumes
# This can be calculated as the total number of segments minus the counterclockwise segments, plus 2
# This calculation weights the intersecting segments equally (i.e. both paths include the intersecting segments)
counterclockwise_segments_count = second_segment - first_segment + 1
total_segments = len(segments)
clockwise_segments_count = total_segments - counterclockwise_segments_count + 2
print("counterclockwise_segments_count: ", counterclockwise_segments_count, "clockwise_segments_count: ", clockwise_segments_count)
plot_polygon_and_paths(points, midpoint_1, midpoint_2, clockwise_points, counterclockwise_points, first_segment, second_segment)
