'''
Identify Disconnected Segments: When you encounter a MultiLineString, this indicates that your line has been broken into multiple segments by the polygon boundary.

Find the Closest Points on the Boundary: For each pair of disconnected line segments, find the closest points on the polygon boundary that connect these two segments.

Create a Boundary Path: Generate a path that follows the boundary of the polygon between these two points.

Concatenate Paths: Concatenate the original line segment, the boundary path, and the next line segment to create a continuous path.

Optional - Smooth the Path: If required, you can smooth the path to avoid sharp turns that are not practical for the robot to navigate.
'''
from shapely.geometry import LineString, MultiLineString
from shapely.ops import nearest_points

def create_boundary_path(polygon, line1_end, line2_start):
    # Find the closest points on the polygon boundary to the end of line1 and the start of line2
    p1, p2 = nearest_points(polygon.boundary, line1_end)
    p3, p4 = nearest_points(polygon.boundary, line2_start)
    
    # Find the indices of these points on the polygon boundary
    boundary_coords = list(polygon.boundary.coords)
    index_p2 = boundary_coords.index((p2.x, p2.y))
    index_p4 = boundary_coords.index((p4.x, p4.y))
    
    # Determine the direction to traverse the boundary points
    if index_p2 < index_p4:
        boundary_path_coords = boundary_coords[index_p2:index_p4+1]
    else:
        boundary_path_coords = boundary_coords[index_p4:index_p2+1][::-1]
    
    # Create a LineString from the boundary path coordinates
    boundary_path = LineString(boundary_path_coords)
    return boundary_path

def concatenate_paths(path1, boundary_path, path2):
    # Combine the paths into a single LineString
    all_coords = list(path1.coords) + list(boundary_path.coords)[1:-1] + list(path2.coords)
    return LineString(all_coords)

def handle_disconnected_segments(trimmed_lines, polygon):
    new_paths = []
    for i in range(len(trimmed_lines) - 1):
        line1 = trimmed_lines[i]
        line2 = trimmed_lines[i + 1]
        boundary_path = create_boundary_path(polygon, line1.boundary[1], line2.boundary[0])
        new_path = concatenate_paths(line1, boundary_path, line2)
        new_paths.append(new_path)
    return new_paths

# You would call this function for each disconnected segment
new_paths = handle_disconnected_segments(disconnected_segments, polygon)
