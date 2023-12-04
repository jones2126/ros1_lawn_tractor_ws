# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_avg_latlon_btwn_2times.py
import rosbag
import utm

#bag_file_path = '/home/tractor/bagfiles/2023-11-30-13-36-28.bag'
bag_file_path = '/home/tractor/bagfiles/Collins_Dr_62_Site_01_run2_2023-11-01-14-51-56.bag'
bag_file = rosbag.Bag(bag_file_path)
start_time = 1
end_time = 95
first_timestamp = None
lat_sum = 0
lon_sum = 0
count = 0
print("Starting....")
for topic, msg, t in bag_file.read_messages(topics=['/fix']):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp

    if start_time <= relative_time <= end_time:  
        lat = msg.latitude
        lon = msg.longitude
        lat_sum += lat
        lon_sum += lon
        count += 1

bag_file.close()

# Calculating the averages
if count > 0:
    avg_lat = lat_sum / count
    avg_lon = lon_sum / count
    print(f"Average Latitude: {avg_lat}, Average Longitude: {avg_lon}")
    print(f"File: {bag_file_path} Starting at: {start_time} Until: {end_time}")
else:
    print("No data found in the specified time range.")
