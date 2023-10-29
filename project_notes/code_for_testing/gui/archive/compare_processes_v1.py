#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/compare_processes_v1.py


import subprocess
import time

def save_ps_to_file(filename):
    """Save the results of 'ps aux' command to a file."""
    with open(filename, 'w') as file:
        subprocess.run(["ps", "aux"], stdout=file)

def launch_start():
    print("In launch_start, opening terminal in 5 seconds")
    time.sleep(5)  # Increased wait for roscore to start
    process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
    return process    

def extract_pid_command_from_ps_file(filename):
    """Extract PID and COMMAND from the results of 'ps aux' saved in a file."""
    with open(filename, 'r') as file:
        lines = file.readlines()[1:]  # Exclude header
        return {(line.split()[1], line.split()[-1]) for line in lines}

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
    baseline_file = '/home/tractor/processes_before.txt'
    save_ps_to_file("processes_before.txt")  # Step 1: Save initial processes
    launch_start()  # launch the start process
    time.sleep(30)
    save_ps_to_file("processes_after.txt")  # Step 3: Wait and save processes after 30 seconds
    compare_and_save_diff(baseline_file, "processes_after.txt", "process_launch_files.txt")  # Step 4: Compare and save the differences
    print("In main, EOJ")

if __name__ == "__main__":
    main()

'''
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


'''