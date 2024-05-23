def find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, output_file_path): 
    def calculate_circle_arc(circle_center, radius, num_points):
        segments = []
        angles = np.linspace(0, 2 * np.pi, num_points + 1)
        for i in range(num_points):
            start_angle = angles[i]
            end_angle = angles[i + 1]
            start_point = (circle_center[0] + radius * np.cos(start_angle), circle_center[1] + radius * np.sin(start_angle))
            end_point = (circle_center[0] + radius * np.cos(end_angle), circle_center[1] + radius * np.sin(end_angle))
            segments.append((start_point, end_point))
        #print("circle segments:", segments)
        return segments

    def do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end):
        def cross_product(a, b):
            return a[0] * b[1] - a[1] * b[0]

        def subtract_vectors(a, b):
            return (a[0] - b[0], a[1] - b[1])

        def is_point_on_segment(seg_start, seg_end, point):
            seg = subtract_vectors(seg_end, seg_start)
            pt_seg = subtract_vectors(point, seg_start)
            cross_prod = cross_product(seg, pt_seg)
            if abs(cross_prod) > np.finfo(float).eps:
                return False  # Point is not on the line created by the segment
            dot_prod = np.dot(seg, pt_seg)
            if dot_prod < 0 or dot_prod > np.dot(seg, seg):
                return False  # Point is outside the segment
            return True

        # Calculate vectors
        r = subtract_vectors(seg1_end, seg1_start)
        s = subtract_vectors(seg2_end, seg2_start)
        qp = subtract_vectors(seg2_start, seg1_start)

        # Calculate cross products and check if line segments are parallel
        r_cross_s = cross_product(r, s)
        qp_cross_r = cross_product(qp, r)

        # If r_cross_s is zero, line segments are parallel or collinear
        if abs(r_cross_s) < np.finfo(float).eps:
            # If qp_cross_r is also zero, line segments are collinear
            if abs(qp_cross_r) < np.finfo(float).eps:
                # Check if the segments overlap
                if is_point_on_segment(seg1_start, seg1_end, seg2_start) or is_point_on_segment(seg1_start, seg1_end, seg2_end) or \
                   is_point_on_segment(seg2_start, seg2_end, seg1_start) or is_point_on_segment(seg2_start, seg2_end, seg1_end):
                    print(seg1_start, seg1_end, seg2_start)
                    return True, (0,0)
            return False, None

        # Calculate the scalar multiples t and u for the intersection point
        t = cross_product(qp, s) / r_cross_s
        u = cross_product(qp, r) / r_cross_s

        # Check if the scalar multiples are within the range [0, 1] for both line segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            # Calculate the intersection point
            intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
            intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
            return True, (intersection_x, intersection_y)
        return False, None

    def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points):
        # use the the circle data, plus the start and end points of the intersecting line segment to calculate the shortest path around the circle

        def adjust_angle(angle):
            """Ensure the angle is within the range [0, 2*pi]."""
            while angle < 0:
                angle += 2 * np.pi
            while angle > 2 * np.pi:
                angle -= 2 * np.pi
            return angle

        def angle_between_points(center, point):
            angle = np.arctan2(point[1] - center[1], point[0] - center[0])
            return angle

        # Determine the angles between the center of the circle and the intersection points 
        start_angle = adjust_angle(angle_between_points(circle_center, start_point))
        end_angle = adjust_angle(angle_between_points(circle_center, end_point))

        # Circumfrence of a circle is 2*Pi().  If the difference in angles > Pi() then that is more than 1/2 the circle which means it is the longer path
        angular_difference = end_angle - start_angle
        if angular_difference > np.pi:
            angular_difference -= 2 * np.pi  # Reverse the angle to get the shorter way
        elif angular_difference < -np.pi:
            angular_difference += 2 * np.pi  # Reverse the angle to get the shorter way

        # Generate points along the shortest arc using the angular difference
        arc_path = []
        for i in range(num_points + 1):
            angle = start_angle + angular_difference * i / num_points
            x = circle_center[0] + radius * np.cos(angle)
            y = circle_center[1] + radius * np.sin(angle)
            arc_path.append((x, y))
        return arc_path

    def plot_circle_and_intersections(center, radius, circle_segments, intersection_points, intersecting_segments, script_name, csv_file_path):
        plt.figure()

        # Plot the circle
        circle = Circle(center, radius, color='green', fill=False)
        plt.gca().add_patch(circle)

        # Plot the circle segments with a single legend entry
        for i, seg in enumerate(circle_segments):
            if i == 0:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-', label='Circle Segments')
            else:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-')

        # Plot the intersection points and label them
        if intersection_points:
            # Offset for text label
            offset = 0.1  # Adjust as needed
            for idx, point in enumerate(intersection_points):
                plt.plot(point[0], point[1], 'ro')
                plt.text(point[0] + offset, point[1] + offset, str(idx + 1), color='black', fontsize=10)

        # Plot the intersecting segments
        if intersecting_segments:
            # Similar approach as above for intersecting segments
            plt.plot([intersecting_segments[0][0][0], intersecting_segments[0][1][0]], 
                     [intersecting_segments[0][0][1], intersecting_segments[0][1][1]], 'y-', label='Intersecting Segments')
            for seg in intersecting_segments[1:]:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'y-')

        plt.title(f'Script: {script_name}\nData source: {csv_file_path}')
        plt.legend()
        plt.axis('equal')

        plt.show()

    def plot_xy_coordinates(csv_file):
        # Load the data from the CSV file
        data = pd.read_csv(csv_file)
        print(f"Rows in file {csv_file} to be plotted: {len(data)}")
        # Plot the x and y coordinates
        plt.figure(figsize=(10, 6))
        plt.autoscale(enable=True, axis='both', tight=None)
        plt.scatter(data['X'], data['Y'], alpha=0.6)
        plt.title('X and Y Coordinates')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')  # Setting equal aspect ratio
        plt.grid(True)
        plt.show()        

    # old version; xy_file_df = pd.read_csv(xy_file_name)      # Read the CSV file

    # Read the text file into a DataFrame using space as the delimiter and naming the columns
    xy_file_df = pd.read_csv(file_path1, delim_whitespace=True, names=['X', 'Y', 'angle', 'lookahead', 'speed'])



    #print(xy_file_df.head(10))
    print(f"Number of rows in DataFrame: {len(xy_file_df)}")
    circle_segments = calculate_circle_arc(circle_center, radius, num_circle_points)  # Calculate circle segments
    print(f"Number of rows in circle_segments: {len(circle_segments)}")

    # Initialize lists to store intersection data
    intersection_points = []
    intersecting_segments = []
    intersecting_index = []
    cir_seg_intersects = []
    current_path_index = None
    x_col_ref = 1
    y_col_ref = 2

    # Iterate through the xy_file_name data
    for i in range(len(xy_file_df) - 1):
        # xy_file_df layout - 'Path_Index' is the reference to which ring of points (i.e. a polygon) the x, y coordinate is a part of 
        #    Path_Index         X         Y
        # 0       0        -5.967672  2.907207

        seg1_start = (xy_file_df.iloc[i, 1], xy_file_df.iloc[i, 2])
        #print("seg1_start:", seg1_start)
        seg1_end = (xy_file_df.iloc[i + 1, 1], xy_file_df.iloc[i + 1, 2])
        cir_seg_ndx = 1
        path_index = xy_file_df.iloc[i, 0]   # Extract the Path_Index for the current row

        if path_index != current_path_index:
            print(f"Looking for intersects in path {path_index}")
            current_path_index = path_index

        for this_segment in circle_segments:
            seg2_start, seg2_end = this_segment
            intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
            if intersects_sw:
                intersection_points.append(intersects_pt)
                intersecting_segments.append((seg1_start, seg1_end))
                intersecting_index.append(i)
                cir_seg_intersects.append(cir_seg_ndx)
            cir_seg_ndx += 1

    # Print the results
    print("Intersection points:", intersection_points)
    print("xy_file_name rows that intersected with the circle:", intersecting_index)
    print("Circle segments with intersects:", cir_seg_intersects)
    plot_circle_and_intersections(circle_center, radius, circle_segments, intersection_points, intersecting_segments, script_name, xy_file_name)

    # Code below rebuilds the path data by using the original DataFrame, but replacing the line segments inside the obstacle/circle with the paths around the circle

    # original_data = pd.read_csv(csv_file_path)                      # Read the CSV file into a DataFrame
    # xy_file_df is the replacement for 'original_data'
    #print(xy_file_df.head(10))
    revised_path = pd.DataFrame()                                   # Initialize an empty DataFrame for revised_path

    if intersecting_index:  # Checks if the list is not empty
        # Beginning Segment
        temp_path = xy_file_df.iloc[0:intersecting_index[0]].copy()
        revised_path = pd.DataFrame()  # or however you initialize it
        temp_path['Source'] = 'temp_path_beg'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        # Middle Segments
        qty_of_intersects = len(intersecting_index) - 2
        print("qty_of_intersects: ", qty_of_intersects)
        print("looping sequence for obstacle path")
        for i, path_index in zip(range(0, qty_of_intersects, 2), range(0, 4)):
            point1 = intersection_points[i]
            point2 = intersection_points[i + 1]
            print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
            circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
            circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
            #circle_arc_df = circle_arc_df.iloc[::-1]  # I need to reverse the order of circle_arc_df so the points run from right to left because I'm running a clockwise pattern
            circle_arc_df['Path_Index'] = path_index
            circle_arc_df['Source'] = 'circle_arc_df'
            revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

            temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:intersecting_index[i+2]].copy()
            temp_path['Path_Index'] = path_index
            temp_path['Source'] = 'temp_path'
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)
        print("End of loop:", i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)

        # Handle End Segment
        print("Handling the end segment")
        path_index = path_index + 1
        i = i + 2
        #print(i, path_index, intersecting_index[i])
        point1 = intersection_points[i]
        point2 = intersection_points[i + 1]
        print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
        circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
        circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
        #circle_arc_df = circle_arc_df.iloc[::-1]  # I need to reverse the order of circle_arc_df so the points run from right to left because I'm running a clockwise pattern
        circle_arc_df['Path_Index'] = path_index
        circle_arc_df['Source'] = 'circle_arc_df_end'
        revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

        temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:].copy()
        temp_path['Path_Index'] = path_index
        temp_path['Source'] = 'end_path'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

    #output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_continuous_obstacles.csv'
    revised_path.to_csv(output_file_path, index=False)              # Save 'revised_path' to the CSV file
    if intersecting_index:  # Checks if the list is not empty
        plot_xy_coordinates(output_file_path)
    return intersection_points, intersecting_segments, intersecting_index, cir_seg_intersects





# 5. Determine the intersection points of the obstacles - at the moment this can only accept one obstacle.  An enhancement is to be able to handle more obstacles.
#additional_circle_center = (17.3, -9.1), additional_radius = 2.4)
circle_center = (17.3, -9.1)  # use rosbag_calc_circle_radius_and_center.py to help with this
radius = 2.4
num_circle_points = 21
xy_file_adjusted_for_obstacles = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_adjusted_for_obstacles.csv'
finished_xy_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt' 

# File path
file_path1 = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/test_generator_output.txt'

intersections, segments, indices, circle_segments = find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, xy_file_adjusted_for_obstacles)
