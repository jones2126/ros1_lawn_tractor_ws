
#!/usr/bin/env python3

import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
import time
import os
import signal

def launch_start():
    print("In launch_start, opening terminal in 10 seconds")
    time.sleep(10)  # Increased wait for roscore to start
    process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
    return process

def launch_stop(process):
    print("In launch_stop, Unregistered the /fix subscriber")
    fix_subscriber.unregister()

    # Find the process started with the command "cub_cadet_real_world_july23.launch"
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

    print("In launch_stop, Pausing 5 seconds")
    time.sleep(5)

def gps_callback(data):
    print(f"In gps_callback, Status: {data.status.status}")

def save_ps_to_file(filename):
    """Save the results of 'ps aux' command to a file."""
    with open(filename, 'w') as file:
        subprocess.run(["ps", "aux"], stdout=file)

def extract_pid_command_from_ps_file(filename):
    """Extract PIDs and commands from a 'ps aux' output file."""
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Skip the header and extract PID and COMMAND columns
    processes = set()
    for line in lines[1:]:
        columns = line.split()
        pid = columns[1]
        cmd = ' '.join(columns[10:])
        processes.add((pid, cmd))
    return processes

def compare_and_save_diff(before_file, after_file, diff_file):
    """Compare two 'ps aux' results and save the differences."""
    before_set = extract_pid_command_from_ps_file(before_file)
    after_set = extract_pid_command_from_ps_file(after_file)
    diff = after_set - before_set
    
    with open(diff_file, 'w') as file:
        file.write("PID\tCOMMAND\n")
        for pid, cmd in diff:
            file.write(f"{pid}\t{cmd}\n")    

def main():
    while True:
        print(f"In main, pausing 5 seconds before starting")
        time.sleep(5)  # Initial wait
        process = launch_start()  # Launch the start method
        print(f"In main, letting ROS finish starting then subscribing to /fix - 15 seconds")
        time.sleep(15)  # Wait
        rospy.init_node('custom_listener', anonymous=True)
        global fix_subscriber  # Make it global so it can be accessed in launch_stop
        fix_subscriber = rospy.Subscriber("/fix", NavSatFix, gps_callback)
        print(f"In main, will close ROS in 10 seconds")
        time.sleep(10)  # Wait
        launch_stop(process)  # Launch the stop method
        print(f"In main, pausing 10 seconds before starting again")
        time.sleep(10)  # Wait before repeating the process

        # Save current processes to a file
        after_file = '/home/tractor/processes_after.txt'
        save_ps_to_file(after_file)

        # Compare before and after files and save the difference
        diff_file_counter = 1  # Initialize a counter for naming difference files
        while True:  # Find an unused filename
            diff_filename = f"/home/tractor/diff_{diff_file_counter}.txt"
            if not os.path.exists(diff_filename):
                break
            diff_file_counter += 1
        compare_and_save_diff('/home/tractor/processes_before.txt', after_file, diff_filename)


if __name__ == "__main__":
    main()


