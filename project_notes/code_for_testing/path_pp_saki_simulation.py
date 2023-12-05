"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_pp_saki_simulation.py
"""
import numpy as np
import math
import matplotlib.pyplot as plt

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle

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

def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    #print(state.x, state.y)
    return state

def PIDControl(target, current):
    a = Kp * (target - current)
    #print(a)
    return a

def pure_pursuit_control(state, cx, cy, pind):
    global complexity_sw
    #print("in pure_pursuit_control")
    #ind = calc_target_index(state, cx, cy)
    ind = simplified_calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha
    if complexity_sw == 1:
        Lf = k * state.v + Lfc
        print("dynamic lookahead = {:.2f}".format(Lf))
        delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    else:
        delta = math.atan2(2.0 * L * math.sin(alpha) / Lfc, 1.0)

 #steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)        

    return delta, ind

'''
This function determines the index of a point on the given path that the robot
should aim for. The purpose of this function is to help the robot look ahead 
on its path rather than simply aiming for the nearest point. 

This function first finds the closest point on the path to the robot and then 
looks further ahead on the path, based on the robot's speed, to determine which 
point it should target next. 

'''
def calc_target_index(state, cx, cy):

    # search nearest point index
    # compute the difference in the x and y coordinates between the robot's current 
    # position and each point on the path.
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    # calculate the distance (d) between the robot and each point on the path. 
    # taking the square root of the sum of the squares of dx and dy (essentially 
    # using the Pythagorean theorem to calculate the Euclidean distance) for each 
    # pair of x and y differences.    
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

    # identifies the index (ind) of the closest point on the path to the robot.
    ind = d.index(min(d))
    L = 0.0

    # Lf represents the look-ahead distance. It's a dynamic distance based on the 
    # robot's current velocity (state.v). 
    Lf = k * state.v + Lfc

    '''
    The purpose of this loop is to find the point on the path that's approximately 
    Lf distance (adjusted look ahead distance) away from the robot's current position. 
    This point becomes the new target for the robot.

    The loop calculates the cumulative distance L from the nearest point (found earlier) 
    while advancing through the path points. The robot increases its target index (ind) 
    until the cumulative distance L surpasses the look-ahead distance Lf or until it 
    reaches the end of the path.

    This approach to determining the look ahead point assumes the list of points
    is sufficient to provide a smooth path.
    '''    
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
       # dy = cx[ind + 1] - cx[ind]  # does not work with vertical lines
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
        #print(dx, dy, L, Lf)
        # Break the loop if we move to a new segment (based on a threshold change in dx or dy)
        #if abs(dx) > 1.5 or abs(dy) > 1.5:
        #    break        

    return ind

def close_enough_to_waypoint(state, waypoint_x, waypoint_y):
    global threshold
    """Check if the robot is close enough to the given waypoint."""
    distance = math.sqrt((state.x - waypoint_x) ** 2 + (state.y - waypoint_y) ** 2)
    return distance < threshold

def simplified_calc_target_index(state, cx, cy):
    global current_waypoint_index
    
    # Check if the robot is close enough to the current target waypoint
    if close_enough_to_waypoint(state, cx[current_waypoint_index], cy[current_waypoint_index]):
        # If we haven't reached the end of the path, move to the next waypoint
        if current_waypoint_index < len(cx) - 1:
            current_waypoint_index += 1
        
    return current_waypoint_index    

def main():
    # define a path  - there are also a couple of example paths at the bottom of the script commented out

    try:
        #with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_output_trimmed.txt', 'r') as file:
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks2_output.txt', 'r') as file:            
            content = file.readlines()

            # Initialize empty lists for x and y coordinates
            cx = []
            cy = []

            for line in content:
                # Split each line by spaces
                parts = line.strip().split()

                # Check if the line has at least two parts (x and y)
                if len(parts) >= 2:
                    # Append x and y values to their respective lists
                    cx.append(float(parts[0]))
                    cy.append(float(parts[1]))
    except Exception as e:
    # Using a general exception to also capture the error message, which can be helpful for debugging
        print(f"File failed to load due to error: {e}")

    
    #print(cx)

    print(type(cx), len(cx))

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    #state = State(x=30.175, y=-96.512, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    #target_ind = calc_target_index(state, cx, cy)
    target_ind = simplified_calc_target_index(state, cx, cy)
    print("taget_ind =", target_ind)
    print("lastIndex =", lastIndex)

    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)
        #print("ai =", ai)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        #print("steer angle = {:.2f}".format(di))
        #distance_to_last_waypoint = math.sqrt((state.x - cx[-1]) ** 2 + (state.y - cy[-1]) ** 2)
        #print(f"Current target_ind: {target_ind}, Distance to last waypoint: {distance_to_last_waypoint:.2f}")
        

        if target_ind == len(cx) - 1 and close_enough_to_waypoint(state, cx[-1], cy[-1]):
            print("Reached the final waypoint!")
            break

        state = update(state, ai, di)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:
        plt.plot(cx, cy, ".r", label="course")
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

    cx = np.concatenate([
        np.linspace(0, edge_length, points_per_side, endpoint=False),  # right
        np.ones(points_per_side) * edge_length,  # up
        np.linspace(edge_length, 0, points_per_side, endpoint=False),  # left
        np.zeros(points_per_side)  # down
    ])
    cy = np.concatenate([
        np.zeros(points_per_side),  # right
        np.linspace(0, edge_length, points_per_side, endpoint=False),  # up
        np.ones(points_per_side) * edge_length,  # left
        np.linspace(edge_length, 0, points_per_side)  # down
    ])

    # A sine wave path - the original from the author
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]    
'''