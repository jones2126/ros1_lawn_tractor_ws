
#!/usr/bin/env python3

import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
import time
import signal

def launch_start():
    print("launch_start")
    process = subprocess.Popen(["roscore"])  # Launch roscore
    time.sleep(10)  # Increased wait for roscore to start
    subprocess.Popen(["roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])
    return process

def launch_stop(process):
    # Unsubscribe from the /fix topic
    fix_subscriber.unregister()
    print("Unregistered the /fix subscriber")

    # Find the process started with the command "cub_cadet_real_world_july23.launch"
    try:
        print("Stopping launch file")
        output = subprocess.check_output(["pgrep", "-f", "cub_cadet_real_world_july23.launch"]).decode().strip()
        if output:
            pids = output.split()
            for pid in pids:
                # Kill the process
                subprocess.run(["kill", "-9", pid])
                print(f"Killed process with PID: {pid} that was started with 'cub_cadet_real_world_july23.launch'")
    except Exception as e:
        print(f"Error while trying to kill process: {e}")

    print("Pausing 5 seconds")
    time.sleep(5)

def gps_callback(data):
    print(f"In gps_callback, Status: {data.status.status}")

def main():
    while True:
        print(f"Countdown to launch roscore - 10 seconds")
        time.sleep(5)  # Initial wait
        process = launch_start()  # Launch the start method
        print(f"Countdown to subscribing to /fix - 15 seconds")
        time.sleep(15)  # Wait
        rospy.init_node('custom_listener', anonymous=True)
        global fix_subscriber  # Make it global so it can be accessed in launch_stop
        fix_subscriber = rospy.Subscriber("/fix", NavSatFix, gps_callback)
        print(f"Countdown to stopping processes - 10 seconds")
        time.sleep(10)  # Wait
        launch_stop(process)  # Launch the stop method
        print(f"Pausing 10 seconds")
        time.sleep(10)  # Wait before repeating the process

if __name__ == "__main__":
    main()
