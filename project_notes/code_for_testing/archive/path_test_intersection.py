
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_test_intersection.py
from sympy import symbols, Eq, solve


def parse_linestring(linestring):
    # Remove the LINESTRING text and parentheses, then split by comma
    points = linestring.replace("LINESTRING ", "").strip("()").split(", ")
    # Split each point by space to get x and y, then convert to float
    points = tuple(tuple(map(float, point.split(" "))) for point in points)
    return points




def find_perpendicular_intersection(line1, line2, from_point=2):
    """
    Calculates the intersection point of a line that is perpendicular to line1 and 
    intersects line2.
    
    Parameters:
    - line1: A tuple of two points (x1, y1) and (x2, y2) representing the first line segment.
    - line2: A tuple of two points (x3, y3) and (x4, y4) representing the second line segment.
    - from_point: A value of 1 or 2 specifying which point the perpendicular should come from on line1 (default is 2).
    
    Returns:
    A tuple (x, y) representing the intersection point.
    """
    # Unpack the line segments
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # Choose the point from which the perpendicular will be drawn
    base_x, base_y = (x1, y1) if from_point == 1 else (x2, y2)

    # Calculate the slope of line1
    m1 = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')

    # Calculate the slope of the perpendicular line
    m2 = -1 / m1 if m1 != 0 else 0

    # Symbols for sympy to solve equations
    x, y = symbols('x y')
    
    # Equation of the perpendicular line through the base point
    eq_perpendicular = Eq((y - base_y), m2 * (x - base_x)) if m1 != float('inf') else Eq(x, base_x)
    
    # Equation of line2
    m_segment = (y4 - y3) / (x4 - x3) if x4 != x3 else float('inf')
    eq_line_segment = Eq((y - y3), m_segment * (x - x3)) if m_segment != float('inf') else Eq(x, x3)
    
    # Solve the system of equations to get the intersection point
    intersection = solve((eq_perpendicular, eq_line_segment), (x, y))

    # Extract intersection point
    intersection_point = (intersection[x], intersection[y])
    return intersection_point

data = [
LINESTRING (-6.373268939109791 -3.8884440636760638, -8.914302735073203 3.4912539271027665)
LINESTRING (-5.195466293158876 -4.837644520732015, -11.07529003323793 12.238603544022125)
LINESTRING (-4.011300363905062 -5.805325294373941, -10.84625596421566 14.044827108479351)
LINESTRING (-2.8596959749679516 -6.678440488434236, -10.470395137867168 15.424634807175941)
LINESTRING (-1.5588777207480118 -7.984904213149015, -10.55621925331021 18.1452729358826)
LINESTRING (0.1359939468964498 -10.435782147516534, -11.314649992756998 22.819302538417748)
LINESTRING (2.140775955664113 -13.786705065678303, -10.869686996106722 23.99842296245285)
LINESTRING (3.502792930443371 -15.27090278034125, -10.215140969318423 24.568870070584882)
LINESTRING (4.381230209445917 -15.3506830823063, -9.534559730718545 25.063705433371123)
LINESTRING (5.163352094532367 -15.150743169572614, -8.859530543951394 25.574665125483367)
]

linestrings = parse_linestrings(data)
# Parse each LINESTRING in the data
lines = [parse_linestring(linestring) for linestring in data]

# Assign to variables
line1 = lines[0]
line2 = lines[1]
line3 = lines[2]
line4 = lines[3]
line5 = lines[4]
line6 = lines[5]
line7 = lines[6]
line8 = lines[7]
line9 = lines[8]
line10 = lines[9]


print("line1:", line1)
print("line2:", line2)


# Get the intersection point from the end of line1
intersection_point_from_end = find_perpendicular_intersection(line1, line2, from_point=2)
print("Intersection point from end of line1:", intersection_point_from_end)

# Get the intersection point from the start of line1
intersection_point_from_start = find_perpendicular_intersection(line2, line3, from_point=1)
print("Intersection point from start of line1:", intersection_point_from_start)
