# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_compare_ll_two_files.py
import rosbag
import matplotlib.pyplot as plt

# Function to extract latitude and longitude data from a rosbag file
def extract_lat_lon(rosbag_path):
    mission_polygon = []
    with rosbag.Bag(rosbag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/fix']):
            if topic == '/fix':
                lat = msg.latitude
                lon = msg.longitude
                mission_polygon.append({'lat': lat, 'lng': lon})
    return mission_polygon

# Paths to your rosbag files
rosbag_path1 = '/home/tractor/bagfiles/2023-11-30-14-39-06.bag'
#rosbag_path2 = '/home/tractor/bagfiles/Collins_Dr_62_Site_01_run1_2023-11-01-14-02-20.bag'
rosbag_path2 = '/home/tractor/bagfiles/Collins_Dr_62_Site_01_run2_2023-11-01-14-51-56.bag'


# Extracting data from the rosbag files
path1 = extract_lat_lon(rosbag_path1)
path2 = extract_lat_lon(rosbag_path2)

# Function to plot the paths
def plot_paths(path1, path2):
    # Extract latitudes and longitudes for each path
    lat1, lon1 = zip(*[(p['lat'], p['lng']) for p in path1])
    lat2, lon2 = zip(*[(p['lat'], p['lng']) for p in path2])

    plt.figure(figsize=(10, 6))

    # Plot each path with a different color
    plt.plot(lon1, lat1, color='blue', linewidth=2, label='Path 1')
    plt.plot(lon2, lat2, color='red', linewidth=2, label='Path 2')

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Paths from ROS Bag Files')
    plt.legend()
    plt.axis('equal') 
    plt.show()

# Plot the paths
plot_paths(path1, path2)
