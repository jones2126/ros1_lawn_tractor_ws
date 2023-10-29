#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/compare_processes.py


import subprocess
import time

def save_ps_to_file(filename):
    """Save the results of 'ps aux' command to a file."""
    with open(filename, 'w') as file:
        subprocess.run(["ps", "aux"], stdout=file)

def launch_start():
    """Run the roslaunch command."""
    process = subprocess.Popen(["roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
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
    # Step 1: Save initial processes
    save_ps_to_file("processes_before.txt")
    
    # Step 2: Launch the desired process
    launch_start()
    
    # Step 3: Wait and save processes after 30 seconds
    time.sleep(30)
    save_ps_to_file("processes_after.txt")
    
    # Step 4: Compare and save the differences
    compare_and_save_diff("processes_before.txt", "processes_after.txt", "process_diff.txt")

if __name__ == "__main__":
    main()
