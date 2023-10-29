
#!/usr/bin/env python3

import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
import time
import signal

def check_and_kill_roscore():
    try:
        print("Checking if roscore is running...")
        pids = subprocess.check_output(["pgrep", "-f", "roscore"]).decode().strip().split()
        current_pid = str(os.getpid())  # Get the current script's PID

        for pid in pids:
            if pid != current_pid:  # Only kill the process if it's not the current script
                print(f"Roscore is running with PID: {pid}. Attempting to kill it...")
                subprocess.run(["kill", "-9", pid])
                time.sleep(5)  # Giving some time for roscore to be killed
    except Exception as e:
        # Catch all exceptions and print them
        print(f"Error in check_and_kill_roscore: {e}")
        pass
    print("check_and_kill_roscore - at the end") 

# This function is based on the extracted launch_start method
def launch_start():
    print("launch_start")
    check_and_kill_roscore()  # Ensure no existing roscore is running
    process = subprocess.Popen(["roscore"])  # Launch roscore
    time.sleep(10)  # Increased wait for roscore to start
    subprocess.Popen(["roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
    return process

# This function is based on the extracted launch_stop method
def launch_stop(process):
    # 1. Unregister the /fix subscriber
    fix_subscriber.unregister()
    print("Unregistered the /fix subscriber")

    rospy.signal_shutdown("GUI Shutdown")
    time.sleep(2)

    # 2. Signal ROS nodes for a graceful shutdown
    if process:
        process.send_signal(signal.SIGINT)
        time.sleep(5)  # Give some time for nodes to shut down gracefully

    # 3. Check for lingering ROS processes and kill if needed
    # (This is a more aggressive approach and can be skipped if not needed)
    lingering_ros_processes = subprocess.check_output(['pgrep', '-f', 'name_of_ros_node_or_script'])
    if lingering_ros_processes:
        subprocess.run(['pkill', '-f', 'name_of_ros_node_or_script'])
        time.sleep(2)  # Wait for processes to be killed

    # 4. Execute rosnode cleanup
    subprocess.run(['rosnode', 'cleanup'])
    print("Executed rosnode cleanup")

    # 5. Terminate the main process (if it's still running)
    if process:
        process.terminate()
        process = None
        print("Terminated the ROS process")

    # 6. Kill roscore
    check_and_kill_roscore()

    # 7. Final delay for cleanup
    time.sleep(5)

# Callback function for the /fix subscriber
def gps_callback(data):
    print(f"In gps_callback, Status: {data.status.status}")

# Main function to orchestrate the required sequence
def main():
    while True:
        print(f"Countdown to launch roscore - 10 seconds")
        time.sleep(10)  # Initial wait
        process = launch_start()  # Launch the start method
        print(f"Countdown to subscribing to /fix - 20 seconds")
        time.sleep(20)  # Wait
        rospy.init_node('custom_listener', anonymous=True)
        global fix_subscriber  # Make it global so it can be accessed in launch_stop
        fix_subscriber = rospy.Subscriber("/fix", NavSatFix, gps_callback)
        print(f"Countdown to stopping processes - 20 seconds")
        time.sleep(20)  # Wait
        launch_stop(process)  # Launch the stop method
        print(f"Pausing 20 seconds")
        time.sleep(20)  # Wait before repeating the process

if __name__ == "__main__":
    main()
