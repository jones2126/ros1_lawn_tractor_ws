# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_get_date_time.py
from path_generator_utilities import get_start_end_times        
bag_path = '/home/tractor/bagfiles/2023-11-03-18-03-02.bag'
start_time, end_time = get_start_end_times(bag_path)
print("Start time:", start_time)
print("End time:", end_time)  