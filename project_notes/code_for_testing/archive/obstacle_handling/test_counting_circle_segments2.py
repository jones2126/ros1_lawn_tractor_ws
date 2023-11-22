# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_counting_circle_segments2.py
def calculate_path_lengths(first_segment, second_segment, total_segments):
    if first_segment > second_segment:
        # Handle wraparound for counterclockwise path
        counterclockwise_segments_count = (second_segment + total_segments) - first_segment + 1
    else:
        counterclockwise_segments_count = second_segment - first_segment + 1

    # Clockwise path is the total segments minus the counterclockwise path, plus 1 for the shared segment
    clockwise_segments_count = total_segments - counterclockwise_segments_count + 2

    return counterclockwise_segments_count, clockwise_segments_count

# Example usage
first_segment = 2
second_segment = 11
total_segments = 21  # Total number of segments in the polygon

counterclockwise_segments_count, clockwise_segments_count = calculate_path_lengths(first_segment, second_segment, total_segments)
print("Counterclockwise segments count:", counterclockwise_segments_count)
print("Clockwise segments count:", clockwise_segments_count)

first_segment = 2
second_segment = 12
total_segments = 21  # Total number of segments in the polygon

counterclockwise_segments_count, clockwise_segments_count = calculate_path_lengths(first_segment, second_segment, total_segments)
print("Counterclockwise segments count:", counterclockwise_segments_count)
print("Clockwise segments count:", clockwise_segments_count)

first_segment = 2
second_segment = 13
total_segments = 21  # Total number of segments in the polygon

counterclockwise_segments_count, clockwise_segments_count = calculate_path_lengths(first_segment, second_segment, total_segments)
print("Counterclockwise segments count:", counterclockwise_segments_count)
print("Clockwise segments count:", clockwise_segments_count)

