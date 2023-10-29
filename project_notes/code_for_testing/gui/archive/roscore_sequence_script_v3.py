
#!/usr/bin/env python3

import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
import time
import signal
import os

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

# ... [Rest of the script remains unchanged] ...

