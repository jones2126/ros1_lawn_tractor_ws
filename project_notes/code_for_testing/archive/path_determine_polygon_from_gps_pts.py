#!/usr/bin/env python3
'''
The script is designed to:
- Read .csv file to get x, y points
- Craft a polygon from those points

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_determine_polygon_from_gps_pts.py

'''
import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")


from path_generator_utilities import load_locations_db, build_points_list, plot_graph

def main():
    file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/paths_locations.txt'   # The path to the locations file
    locations_data = load_locations_db(file_path) 
    site = 'Collins_Dr_62_Site_01'
    for location in locations_data:
        if location['location_name'] == site:
            csv_file_path = location['csv_file_path']
            origin_lat = location['origin_lat']
            origin_lon = location['origin_lon']
            print(f"path and longitude: {location['csv_file_path']}, Longitude: {location['origin_lon']}")
    print(f"reading file: {csv_file_path}")
    points, num_points = build_points_list(csv_file_path, 'X', 'Y')
    print(f"points returned: {num_points}")
    #plot_graph(points, keep_plot_open='N')
    plot_graph(points, script_name, csv_file_path, keep_plot_open='Y', annotate_points='N') 

if __name__ == '__main__':
    main()    