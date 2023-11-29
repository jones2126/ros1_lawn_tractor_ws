# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_generator_using_4_pts.py
def calculate_speed(meters, seconds):
    if seconds == 0:
        return "Duration cannot be zero."
    
    # Speed in meters per second
    speed_m_s = meters / seconds

    # Conversion constants
    meters_in_a_mile = 1609.34
    seconds_in_an_hour = 3600
    
    # Speed in miles per hour
    speed_mph = (meters / meters_in_a_mile) * (seconds_in_an_hour / seconds)

    return speed_mph, speed_m_s

# Example usage:
meters_traveled = 1413  # replace with actual meters traveled
duration_seconds = 2441  # replace with actual duration in seconds


points = [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]

# Get the last four elements
inner_points = points[-4:]
print("Last 4 points:", inner_points)
inner_points.append(inner_points[0])
print("closed:", inner_points)


mph, m_s = calculate_speed(meters_traveled, duration_seconds)
print(f"Speed: {mph} MPH, {m_s} m/s")