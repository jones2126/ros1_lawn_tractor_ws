#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/compare_processes_v2.py


import subprocess
import time
import re
import rospy
from sensor_msgs.msg import NavSatFix

fix_subscriber = None

def save_ps_to_file(filename):
    """Save the results of 'ps aux' command to a file."""
    with open(filename, 'w') as file:
        subprocess.run(["ps", "aux"], stdout=file)
        print(f"In save_ps_to_file, Results saved to {filename}")

def launch_start():
    print("In launch_start, opening terminal in 5 seconds")
    time.sleep(5)  # Increased wait for roscore to start
    process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
    print("In launch_start, Pausing 10 seconds")
    time.sleep(10)

    # Initialize node and subscribe to the /fix topic
    global fix_subscriber
    rospy.init_node('gps_listener', anonymous=True)
    fix_subscriber = rospy.Subscriber("/fix", NavSatFix, gps_callback)
    print("In launch_start, After /fix subscribing....Pausing 20 seconds")
    time.sleep(20)
    print("In launch_start, done sleepping")
    return process    

def extract_pid_command_from_ps_file(filename):
    """Extract PID and COMMAND from the results of 'ps aux' saved in a file."""
    with open(filename, 'r') as file:
        lines = file.readlines()[1:]    # Exclude header from the file
        pid_command_set = set()         # Initialize an empty set
        for line in lines:              # Iterate over each line and add the PID and COMMAND pairs to the set
            split_line = re.split(r'\s+', line.strip())
            pid = split_line[1]
            command = " ".join(split_line[10:])
            pid_command_set.add((pid, command))
        return pid_command_set

def compare_and_save_diff(before_file, after_file, diff_file):
    """Compare two 'ps aux' results and save the differences."""
    before_set = extract_pid_command_from_ps_file(before_file)
    after_set = extract_pid_command_from_ps_file(after_file)
    diff = after_set - before_set
    
    with open(diff_file, 'w') as file:
        file.write("PID\tCOMMAND\n")
        for pid, cmd in diff:
            file.write(f"{pid}\t{cmd}\n")

def launch_stop(process):
    print("In launch_stop")
    global fix_subscriber
    fix_subscriber.unregister()
    print("Unregistered the /fix subscriber, Pausing 15 seconds")
    time.sleep(15)
    #rospy.signal_shutdown("GUI Shutdown")
    #time.sleep(2)

    # Find the PID for "cub_cadet_real_world_july23.launch"
    try:
        print("In launch_stop, Stopping launch file")
        output = subprocess.check_output(["pgrep", "-f", "cub_cadet_real_world_july23.launch"]).decode().strip()
        if output:
            pids = output.split()
            for pid in pids:
                # Kill the process
                subprocess.run(["kill", "-9", pid])
                print(f"Killed process with PID: {pid} that was started with 'cub_cadet_real_world_july23.launch'")
    except Exception as e:
        print(f"Error while trying to kill process: {e}")

    print("In launch_stop, Pausing 15 seconds")
    time.sleep(15)            

def kill_ros_processes(process_list):
    # Kill each ROS process
    for pid in process_list:
        try:
            subprocess.run(["kill", "-9", pid])
            print(f"Killed process with PID: {pid}")
        except Exception as e:
            print(f"Error killing process with PID {pid}. Error: {e}")

def gps_callback(data):
    # Update the GPS status based on the incoming message
    gps_status = data.status.status
    print(f"GPS Status: {gps_status}")



def main():
    # if you have a baseline file to use update the "compare_and_save_diff" statement
    save_ps_to_file("processes_before.txt")  # Save initial processes
    process = launch_start()  # Launch the start method    
    save_ps_to_file("processes_after_start.txt") 
    compare_and_save_diff("processes_before.txt", "processes_after_start.txt", "processes_started.txt")  # Which processes started
    launch_stop(process)  # Launch the stop method
    save_ps_to_file("processes_post_stop.txt")       # take a snapshot of running processes
    compare_and_save_diff("processes_before.txt", "processes_post_stop.txt", "processes_diff_post_stop.txt") 
    with open('processes_diff_post_stop.txt', 'r') as file:
        data_content = file.read()
    lines = data_content.strip().split("\n")[1:]  # Split by lines and exclude the header
    process_list = {line.split("\t")[0] for line in lines if '__log:=' in line.split("\t")[1]}  # Extract PID if '__log:=' is in COMMAND
    #print("printing process_list", process_list)
    kill_ros_processes(process_list)
    save_ps_to_file("processes_post_stop_post_log.txt")
    compare_and_save_diff("processes_before.txt", "processes_post_stop_post_log.txt", "processes_diff_post_stop_post_log.txt")
    with open("processes_diff_post_stop_post_log.txt", 'r') as file:
        content = file.read()
        print(content)
    print("In main, EOJ")

if __name__ == "__main__":
    i = 1
    while i < 5:
        main()
        i += 1
'''

'''