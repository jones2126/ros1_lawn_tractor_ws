#!/usr/bin/env python3

def load_mission_data():
    '''
    This function assumes the 'mission' is a series of waypoints to achieve
    one after another and the file is in the format:
    x position, space, y position, space, yaw

    This is the way the program 'path_planning_waypoint_generator.py' creates
    paths that follow a dubins path.
    '''
    init_x = 1
    init_y = 1
    with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks.txt', 'r') as file:
        content = file.readlines()

        # Convert content into a list of (x, y) pairs
        coordinates = [(float(line.split()[0]), float(line.split()[1])) for line in content]

        # Initialize the mission list with the robot's current position and the first coordinate from the file
        #mission = [(init_x, init_y, coordinates[0])]
        #mission = [((init_x, init_y), coordinates[0])]
        mission = []

        # Add pairs of consecutive coordinates to the mission list
        for i in range(len(coordinates) - 1):
            mission.append((coordinates[i], coordinates[i+1]))

    return mission
if __name__ == '__main__':
    print("starting the load test")
    # Load the mission data
    mission = load_mission_data()
    print(type(mission), len(mission))
    print (mission)