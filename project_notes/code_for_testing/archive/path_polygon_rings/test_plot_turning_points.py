# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_plot_turning_points.py
from shapely.geometry import Polygon, LineString, Point
from shapely.affinity import translate
import matplotlib.pyplot as plt
import numpy as np
import shapely

print(shapely.__version__)

def print_interior_points(polygon):
  """
  Prints the coordinates of all interior points within the polygon.
  """
  interior_points = list(polygon.interior.coords)
  print(f"Interior points: {interior_points}")



def visualize_translation(original_polygon, translated_polygon):
  """
  Visualizes the original and translated polygon using matplotlib.

  Args:
    original_polygon: Shapely Polygon object representing the original polygon.
    translated_polygon: Shapely Polygon object representing the translated polygon.
  """
  fig, ax = plt.subplots()

  # Plot the original polygon in blue
  ax.plot(*original_polygon.exterior.coords.xy, color='blue', alpha=0.7, label='Original Polygon')

  # Plot the translated polygon in red
  ax.plot(*translated_polygon.exterior.coords.xy, color='red', alpha=0.7, label='Translated Polygon')

  # Set axis labels and title
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_title('Original and Translated Polygons')

  # Show the legend and plot
  ax.legend()
  plt.show()

def calculate_turning_points(polygon_points, turning_radius, path_width):
    field_polygon = Polygon(polygon_points)
    current_polygon = field_polygon
    turning_points = []
    direction = 1  # Start with eastward direction

    while True:
        old_bounds = current_polygon.bounds
        print("bounds (before translation)", old_bounds)
        print("Current polygon address (before):", id(current_polygon))
        off_set = -path_width * direction
        print("off_set", off_set)
        original_polygon = Polygon(polygon_points)
        translated_polygon = translate(original_polygon, off_set, 0)
        visualize_translation(original_polygon, translated_polygon)
        # Print interior points after translation
        #print_interior_points(translated_polygon)
        print("Translated polygon coordinates:", translated_polygon.exterior.coords)
        current_polygon = translated_polygon        
        print("current_polygon coordinates:", current_polygon.exterior.coords)
        print("Current polygon address (after):", id(current_polygon))
        new_bounds = current_polygon.bounds
        effective_width = new_bounds[2] - new_bounds[0]
        print("Effective width:", effective_width)        
        print("bounds (after translation)", new_bounds)
        if effective_width < 2 * path_width:
            break
        print("effective_width", effective_width)
        print("direction", direction)
        if direction == 1:  # Eastward
            x_coord = new_bounds[0] + turning_radius
        else:  # Westward
            x_coord = new_bounds[2] - turning_radius
        line = LineString([(x_coord, new_bounds[1]), (x_coord, new_bounds[3])])
        intersection = current_polygon.intersection(line)
        print("Line x-coordinate:", x_coord)
        print("Line bounds:", line.bounds)
        if not intersection.is_empty:
            print("Intersection point:", intersection.coords[0])
            current_bounds = current_polygon.bounds
            end_point = intersection.coords[0]
            turn_start = (x_coord, end_point[1])
            turn_end = (x_coord, end_point[1] + path_width * direction)
            turning_points.append(turn_start)
            turning_points.append(turn_end)
        bounds = new_bounds
        direction *= -1
        input("press enter")

    return turning_points

# Example usage
polygon_points = [(0, 0), (10, 0), (10, 10), (0, 10)]  # Define the corners of your field
turning_radius = 1.5
path_width = 0.9

turning_points = calculate_turning_points(polygon_points, turning_radius, path_width)
print(turning_points)




# Plotting with corrected turning points
x, y = zip(*polygon_points)
plt.figure(figsize=(10, 8))
plt.plot(x + (x[0],), y + (y[0],), 'b-', label='Field Boundary')  # Plot field boundary

# Plot turning points
for i, tp in enumerate(turning_points):
    plt.plot(tp[0], tp[1], 'ro')
    plt.text(tp[0], tp[1], f'{i}', fontsize=12, color='black', ha='right')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Field Coverage Path with Turning Points')
plt.legend()
plt.grid(True)
plt.show()