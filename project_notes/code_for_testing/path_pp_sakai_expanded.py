"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
Changes by Al Jones, added 'calcWaypointIndex', variable names expanded, comments added for understanding


$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_pp_saki_simulation.py
"""
import numpy as np
import math
import matplotlib.pyplot as plt

k = 0.1  # look forward gain
lookahead_distance = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
wheelbase = 2.9  # [m] wheel base of vehicle

show_animation = True

# Global variable to keep track of the last waypoint that the robot was tracking
current_waypoint_index = 0
complexity_sw = 0  # used to disinguish whether to adjust lookahead based on speed
threshold = .3

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
    #def __init__(self, x=30.175, y=-96.512, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def update(state, a, UsteerAngle):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    # yaw represents the desired orientation that the robot should have based on the calculated steering angle.
    state.yaw = state.yaw + state.v / wheelbase * math.tan(UsteerAngle) * dt
    state.v = state.v + a * dt
    #print(state.x, state.y)
    return state

def PIDControl(target, current):
    a = Kp * (target - current)
    #print(a)
    return a

def pure_pursuit_control(state, mission_list_x_coordinates, mission_list_y_coordinates, inputWaypointIndex):
    global complexity_sw
    #print("in pure_pursuit_control")
    #ppcWaypointIndex = calcWaypointIndexOrig(state, mission_list_x_coordinates, mission_list_y_coordinates)
    ppcWaypointIndex = calcWaypointIndex(state, mission_list_x_coordinates, mission_list_y_coordinates)

    if inputWaypointIndex >= ppcWaypointIndex:
        ppcWaypointIndex = inputWaypointIndex

    if ppcWaypointIndex < len(mission_list_x_coordinates):
        tx = mission_list_x_coordinates[ppcWaypointIndex]
        ty = mission_list_y_coordinates[ppcWaypointIndex]
    else:
        tx = mission_list_x_coordinates[-1]
        ty = mission_list_y_coordinates[-1]
        ppcWaypointIndex = len(mission_list_x_coordinates) - 1

    angleToGoal = math.atan2(ty - state.y, tx - state.x) - state.yaw
    '''

    '''

    if state.v < 0:  # back
        angleToGoal = math.pi - angleToGoal
    if complexity_sw == 1:
        speed_adjusted_lookahead_distance = k * state.v + lookahead_distance
        print("dynamic lookahead = {:.2f}".format(speed_adjusted_lookahead_distance))
        str_angl = math.atan2(2.0 * wheelbase * math.sin(angleToGoal) / speed_adjusted_lookahead_distance, 1.0)
    else:
        str_angl = math.atan2(2.0 * wheelbase * math.sin(angleToGoal) / lookahead_distance, 1.0)

 #steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)        

    return str_angl, ppcWaypointIndex

'''
This function determines the index of a point on the given path that the robot should aim for. The purpose of this 
function is to help the robot look ahead on its path rather than simply aiming for the nearest point. 

This function first finds the closest point on the path to the robot and then looks further ahead on the path, 
based on the robot's speed, to determine which point it should target next. 

'''
def calcWaypointIndexOrig(state, mission_list_x_coordinates, mission_list_y_coordinates):

    # search nearest point index
    # computes the difference between the robot's current x-coordinate (state.x) and each mission waypoint's x-coordinate. 
    # The result is a list of differences in the x-coordinates, stored in dx.
    dx = [state.x - x_coordinate for x_coordinate in mission_list_x_coordinates]
    dy = [state.y - y_coordinate for y_coordinate in mission_list_y_coordinates]

    # calculate the distance (d) between the robot and each point on the path. 
    # taking the square root of the sum of the squares of dx and dy (essentially 
    # using the Pythagorean theorem to calculate the Euclidean distance) for each 
    # pair of x and y differences.    
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    # identifies the index (ind) of the closest point on the path to the robot.
    ind = d.index(min(d))
    cum_dist_frm_nearst_pt = 0.0

    # speed_adjusted_lookahead_distance represents the look-ahead distance. It's a dynamic distance based on the 
    # robot's current velocity (state.v). 
    speed_adjusted_lookahead_distance = k * state.v + lookahead_distance

    '''
    The purpose of this loop is to find the point on the path that's approximately 
    speed_adjusted_lookahead_distance distance away from the robot's current position. 
    This point becomes the new target for the robot.

    The loop calculates 'cum_dist_frm_nearst_pt' the cumulative distance from the nearest point found earlier
    while advancing through the path points. The robot increases its target index (ind) 
    until cum_dist_frm_nearst_pt surpasses speed_adjusted_lookahead_distance or until it reaches the end of the path.

    This approach to determining the look ahead point assumes the list of points is sufficient to provide a smooth path.
    '''    
    while speed_adjusted_lookahead_distance > cum_dist_frm_nearst_pt and (ind + 1) < len(mission_list_x_coordinates):
        dx = mission_list_x_coordinates[ind + 1] - mission_list_x_coordinates[ind]
        dy = mission_list_y_coordinates[ind + 1] - mission_list_y_coordinates[ind]
       # dy = mission_list_x_coordinates[ind + 1] - mission_list_x_coordinates[ind]  # does not work with vertical lines
        cum_dist_frm_nearst_pt += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
        #print(dx, dy, cum_dist_frm_nearst_pt, speed_adjusted_lookahead_distance)
        # Break the loop if we move to a new segment (based on a threshold change in dx or dy)
        #if abs(dx) > 1.5 or abs(dy) > 1.5:
        #    break        

    return ind


def withinThreshold(state, waypoint_x, waypoint_y):
    # checks if robot and current waypoint is within the threshold and sets true/false condition
    global threshold
    distance = math.sqrt((state.x - waypoint_x) ** 2 + (state.y - waypoint_y) ** 2)
    return distance < threshold

def calcWaypointIndex(state, mission_list_x_coordinates, mission_list_y_coordinates):
    global current_waypoint_index
    
    # Check if the robot is close enough to the current target waypoint
    if withinThreshold(state, mission_list_x_coordinates[current_waypoint_index], mission_list_y_coordinates[current_waypoint_index]):
        # If we haven't reached the end of the path, move to the next waypoint
        if current_waypoint_index < len(mission_list_x_coordinates) - 1:
            current_waypoint_index += 1
        
    return current_waypoint_index   


def main():
    # define a path  - there are also a couple of example paths at the bottom of the script commented out

    try:
        #with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_output_trimmed.txt', 'r') as file:
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks2_output.txt', 'r') as file:            
            content = file.readlines()

            # Initialize empty lists for x and y coordinates
            mission_list_x_coordinates = []
            mission_list_y_coordinates = []

            for line in content:
                # Split each line by spaces
                parts = line.strip().split()

                # Check if the line has at least two parts (x and y)
                if len(parts) >= 2:
                    # Append x and y values to their respective lists
                    mission_list_x_coordinates.append(float(parts[0]))
                    mission_list_y_coordinates.append(float(parts[1]))
    except Exception as e:
    # Using a general exception to also capture the error message, which can be helpful for debugging
        print(f"File failed to load due to error: {e}")

    
    #print(mission_list_x_coordinates)

    print(type(mission_list_x_coordinates), len(mission_list_x_coordinates))

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    #state = State(x=30.175, y=-96.512, yaw=0.0, v=0.0)

    lastWaypointIndex = len(mission_list_x_coordinates) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    #waypointIndex = calcWaypointIndexOrig(state, mission_list_x_coordinates, mission_list_y_coordinates)
    waypointIndex = calcWaypointIndex(state, mission_list_x_coordinates, mission_list_y_coordinates)
    print("waypointIndex =", waypointIndex)
    print("lastWaypointIndex =", lastWaypointIndex)

    while T >= time and lastWaypointIndex > waypointIndex:
        ai = PIDControl(target_speed, state.v)
        #print("ai =", ai)
        steerAngle, waypointIndex = pure_pursuit_control(state, mission_list_x_coordinates, mission_list_y_coordinates, waypointIndex)
        #print("steer angle = {:.2f}".format(steerAngle))
        #distance_to_last_waypoint = math.sqrt((state.x - mission_list_x_coordinates[-1]) ** 2 + (state.y - mission_list_y_coordinates[-1]) ** 2)
        #print(f"Current waypointIndex: {waypointIndex}, Distance to last waypoint: {distance_to_last_waypoint:.2f}")
        

        if waypointIndex == len(mission_list_x_coordinates) - 1 and withinThreshold(state, mission_list_x_coordinates[-1], mission_list_y_coordinates[-1]):
            print("Reached the final waypoint!")
            # Stop moving.
            cmd_vel.linear.x = 0.0
            steering_angle = 0.0            
            break

        state = update(state, ai, steerAngle)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(mission_list_x_coordinates, mission_list_y_coordinates, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(mission_list_x_coordinates[waypointIndex], mission_list_y_coordinates[waypointIndex], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastWaypointIndex >= waypointIndex, "Cannot goal"

    if show_animation:
        plt.plot(mission_list_x_coordinates, mission_list_y_coordinates, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()

'''
    # Define square path with a point every meter
    edge_length = 20.0  # length of one side of the square in meters
    points_per_side = int(edge_length)  # 20 points for each side

    mission_list_x_coordinates = np.concatenate([
        np.linspace(0, edge_length, points_per_side, endpoint=False),  # right
        np.ones(points_per_side) * edge_length,  # up
        np.linspace(edge_length, 0, points_per_side, endpoint=False),  # left
        np.zeros(points_per_side)  # down
    ])
    mission_list_y_coordinates = np.concatenate([
        np.zeros(points_per_side),  # right
        np.linspace(0, edge_length, points_per_side, endpoint=False),  # up
        np.ones(points_per_side) * edge_length,  # left
        np.linspace(edge_length, 0, points_per_side)  # down
    ])

    # A sine wave path - the original from the author
    mission_list_x_coordinates = np.arange(0, 50, 0.1)
    mission_list_y_coordinates = [math.sin(ix / 5.0) * ix / 2.0 for ix in mission_list_x_coordinates]    
'''