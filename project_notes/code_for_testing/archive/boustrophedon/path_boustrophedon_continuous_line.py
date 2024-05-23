# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_boustrophedon_continuous_line.py

import matplotlib.pyplot as plt
from math import sqrt, atan2, pi

# Function to normalize a vector
def normalize(vector):
    norm = sqrt(vector[0]**2 + vector[1]**2)
    return (vector[0]/norm, vector[1]/norm)

# Function to calculate a perpendicular vector
def perpendicular(vector, direction='right'):
    if direction == 'right':
        return (-vector[1], vector[0])
    else:
        return (vector[1], -vector[0])

# Function to calculate the intersection point of two lines
def intersection_point(p1, p2, p3, p4):
    denominator = ((p4[0] - p3[0]) * (p2[1] - p1[1])) - ((p4[1] - p3[1]) * (p2[0] - p1[0]))
    if denominator == 0:
        return None  # Lines are parallel
    t = ((p3[0] - p1[0]) * (p4[1] - p3[1])) - ((p3[1] - p1[1]) * (p4[0] - p3[0]))
    t /= denominator
    return (p1[0] + (t * (p2[0] - p1[0])), p1[1] + (t * (p2[1] - p1[1])))

# Function to determine the turn direction
def turn_direction(current_direction, next_direction):
    angle = atan2(next_direction[1], next_direction[0]) - atan2(current_direction[1], current_direction[0])
    angle = angle % (2 * pi)
    return 'right' if angle < pi else 'left'

# Function to create the continuous path
def create_continuous_path(path_data):
    continuous_path = []
    for i in range(len(path_data)):
        # Extract the start and end points of the current segment
        start_point = path_data[i][0]
        end_point = path_data[i][1]

        # Add the start point to the continuous path
        if i == 0:
            continuous_path.append(start_point)

        # Determine the turn direction
        if i < len(path_data) - 1:
            direction = 'right' if i % 2 == 0 else 'left'
            # Calculate the perpendicular direction
            current_direction = normalize((end_point[0] - start_point[0], end_point[1] - start_point[1]))
            perp_direction = perpendicular(current_direction, direction)
            next_start = path_data[i + 1][0]
            next_end = path_data[i + 1][1]
            # Find the intersection point
            intersect = intersection_point(end_point, (end_point[0] + perp_direction[0], end_point[1] + perp_direction[1]), next_start, next_end)
            if intersect:
                continuous_path.append(intersect)

        # Add the end point to the continuous path
        continuous_path.append(end_point)

    return continuous_path

# Parse the linestrings to extract the coordinates
def parse_linestring(linestring):
    coords = linestring.strip('LINESTRING ()').split(', ')
    coords = [(float(x.split(' ')[0]), float(x.split(' ')[1])) for x in coords]
    return coords

# Example LINESTRING data from your path
path_data = [
"LINESTRING (-6.373268939109791 -3.8884440636760638, -8.914302735073203 3.4912539271027665)",
"LINESTRING (-5.195466293158876 -4.837644520732015, -11.07529003323793 12.238603544022125)",
"LINESTRING (-4.011300363905062 -5.805325294373941, -10.84625596421566 14.044827108479351)",
"LINESTRING (-2.8596959749679516 -6.678440488434236, -10.470395137867168 15.424634807175941)",
"LINESTRING (-1.5588777207480118 -7.984904213149015, -10.55621925331021 18.1452729358826)"
]




# Convert the linestrings to a list of segment start and end points
segment_data = [parse_linestring(linestring) for linestring in path_data]

# Create the continuous path
continuous_path = create_continuous_path(segment_data)

# Plot the continuous path
x_coords, y_coords = zip(*continuous_path)  # Unpack the list of tuples

plt.figure(figsize=(10, 5))
plt.plot(x_coords, y_coords, marker='o')  # Plot the path with markers for each point
plt.title('Robot Toolpath')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.grid(True)
plt.axis('equal')  # Ensuring equal scaling for x and y axes
plt.show()
