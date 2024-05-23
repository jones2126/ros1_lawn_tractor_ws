#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/gui_main_v1.py
import tkinter as tk
import tkinter.font as tkFont 
import subprocess
import rospy
import socket
from sensor_msgs.msg import NavSatFix
import time
import signal
import re
import os



class ROSLauncher(tk.Tk):

    def __init__(self):
        super().__init__()
        self.process = None  # Variable to hold the ROS process
        self.last_gps_update = 0


        # Get screen dimensions
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()

        # Set window size and center it
        screen_proportion = 0.85
        width = int(screen_width * screen_proportion)
        height = int(screen_height * screen_proportion)
        x_offset = int((screen_width - width) / 2)
        y_offset = int((screen_height - height) / 2)

        # Define the button font
        self.button_font = tkFont.Font(size=28, weight='bold')  

        self.geometry(f"{width}x{height}+{x_offset}+{y_offset}")

        # Prevent window from being resized
        self.resizable(False, False)

        # Create and layout buttons
        self.create_button("Start", self.launch_start).grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 1", self.launch_location1).grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        self.create_button("New Path", self.launch_drive_new_path).grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 2", self.launch_location2).grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        self.create_button("Stop", self.launch_stop).grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 3", self.launch_location3).grid(row=2, column=1, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 4", self.launch_location4).grid(row=3, column=1, sticky="nsew", padx=5, pady=5)

        # GPS status visualization (icon)
        self.gps_status_canvas = tk.Canvas(self, width=50, height=50, bg="red", highlightthickness=0)
        self.gps_status_canvas.grid(row=3, column=0, sticky="sw", padx=5, pady=5)        

        # Configure rows and columns to expand
        for i in range(4):
            self.grid_rowconfigure(i, weight=1)

        for i in range(3):
            self.grid_columnconfigure(i, weight=1)

        # Start the periodic checks
        #self.periodic_check_ros()
        self.check_gps_timeout()

    def check_ros_master(self):
        try:
            s = socket.create_connection(("localhost", 11311), timeout=1)
            s.close()
            return True
        except socket.error:
            return False

    def extract_pid_command_from_ps_file(self, filename):
        """Extract PID and COMMAND from the results of 'ps aux' saved in a file."""
        with open(filename, 'r') as file:
            lines = file.readlines()[1:]  # Exclude header
            return {(line.split()[1], line.split()[-1]) for line in lines}

    def compare_and_save_diff(self, before_file, after_file, diff_file):
        """Compare two 'ps aux' results and save the differences."""
        before_set = self.extract_pid_command_from_ps_file(before_file)
        after_set = self.extract_pid_command_from_ps_file(after_file)
        diff = after_set - before_set
        
        with open(diff_file, 'w') as file:
            file.write("PID\tCOMMAND\n")
            for pid, cmd in diff:
                file.write(f"{pid}\t{cmd}\n")                    

    def periodic_check_ros(self):
        print(f"In periodic_check_ros")
        if self.check_ros_master():
            rospy.init_node('gui_listener', anonymous=True)
            self.fix_subscriber = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
            print(f"In periodic_check_ros, subscribing to /fix")
            self.after(5000, self.periodic_check_ros) 
        else:
            self.after(5000, self.periodic_check_ros) 

    def on_button_hover(self, event):
        event.widget.config(bg="#FF0000")  # red

    def on_button_hover_out(self, event):
        event.widget.config(bg="#D3D3D3")  # grey
        
    def create_button(self, text, command):
        button = tk.Button(self, text=text, command=command, font=self.button_font, bg="#D3D3D3", activebackground="#FF0000")
        button.bind("<Enter>", self.on_button_hover)
        button.bind("<Leave>", self.on_button_hover_out)
        return button

    def save_ps_to_file(self, filename):
        """Save the results of 'ps aux' command to a file."""
        with open(filename, 'w') as file:
            subprocess.run(["ps", "aux"], stdout=file)
            print(f"In save_ps_to_file, Results saved to {filename}")

    def extract_pid_command_from_ps_file(self, filename):
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
   
    def launch_start(self):
        print("In launch_start, saving processes in 5 seconds")
        time.sleep(5)  # Increased wait for roscore to start
        self.save_ps_to_file("processes_before.txt")  # Save initial processes
        print("In launch_start, launching cub_cadet_real_world_oct23.launch in 5 seconds")
        time.sleep(5)  # Increased wait for roscore to start
        self.process = subprocess.Popen(["gnome-terminal", "--", "roslaunch", "ackermann_vehicle", "cub_cadet_real_world_oct23.launch"])      
        print("In launch_start, gps_listener in 10 seconds")
        time.sleep(10)      
        # 
        # the function gps_callback which is setup below is used to update the GUI screen with the GPS status so I know
        # when RTK fix is achieved
        #
        rospy.init_node('gps_listener', anonymous=True)
        print("In launch_start, about to subscribe /fix in 10 seconds")
        time.sleep(10)    
        self.fix_subscriber = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        print("In launch_start, After /fix subscribing....Pausing 10 seconds")
        time.sleep(10)
        print("In launch_start, done sleepping")

    def launch_stop(self):
        print("In launch_stop")
        try:
            print("Stopping rosbag recording")
            output = subprocess.check_output(["pgrep", "-f", "rosbag record -a"]).decode().strip()
            if output:
                pids = output.split()
                for pid in pids:
                    # Send SIGINT to gracefully stop the rosbag recording
                    subprocess.run(["kill", "-2", pid])
                    print(f"Stopped rosbag recording process with PID: {pid}")
        except Exception as e:
            print(f"Error while trying to stop rosbag recording: {e}")
        time.sleep(2)

        try:
            print("Closing the rosbag window")
            output = subprocess.check_output(["pgrep", "-f", "/usr/libexec/gnome-terminal-server"]).decode().strip()
            if output:
                pids = output.split()
                for pid in pids:
                    # Send SIGINT to gracefully stop the rosbag recording
                    subprocess.run(["kill", "-9", pid])
                    print(f"Stopped rosbag recording process with PID: {pid}")
        except Exception as e:
            print(f"Error while trying to stop rosbag recording: {e}")
        time.sleep(2)

        self.fix_subscriber.unregister()
        print("Unregistered the /fix subscriber, Pausing 5 seconds")
        time.sleep(5)

        # Find the PID for "ackermann_vehicle cub_cadet_real_world_oct23.launch"
        try:
            print("In launch_stop, Stopping launch file")
            output = subprocess.check_output(["pgrep", "-f", "ackermann_vehicle cub_cadet_real_world_oct23.launch"]).decode().strip()
            if output:
                pids = output.split()
                for pid in pids:
                    # Kill the process
                    subprocess.run(["kill", "-9", pid])
                    print(f"Killed process with PID: {pid} that was started with 'ackermann_vehicle cub_cadet_real_world_oct23.launch'")
        except Exception as e:
            print(f"Error while trying to kill process: {e}")

        # Find the PID for "pure_pursuit_cub_cadet_oct23.launch"
        try:
            print("In launch_stop, Stopping launch file")
            output = subprocess.check_output(["pgrep", "-f", "pure_pursuit_cub_cadet_oct23.launch"]).decode().strip()
            if output:
                pids = output.split()
                for pid in pids:
                    # Kill the process
                    subprocess.run(["kill", "-9", pid])
                    print(f"Killed process with PID: {pid} that was started with 'pure_pursuit_cub_cadet_oct23.launch'")
        except Exception as e:
            print(f"Error while trying to kill process: {e}")            

        print("In launch_stop, Pausing 10 seconds")
        time.sleep(10)            

        self.save_ps_to_file("processes_post_stop.txt")       # take a snapshot of running processes

        before_set = self.extract_pid_command_from_ps_file("processes_before.txt")
        after_set = self.extract_pid_command_from_ps_file("processes_post_stop.txt")
        diff = after_set - before_set
        
        with open("processes_diff_post_stop.txt", 'w') as file:
            file.write("PID\tCOMMAND\n")
            for pid, cmd in diff:
                file.write(f"{pid}\t{cmd}\n")    

        # get the PID numbers that need to be stopped and execute a 'kill' command for each of them
        with open('processes_diff_post_stop.txt', 'r') as file:
            data_content = file.read()
        lines = data_content.strip().split("\n")[1:]  # Split by lines and exclude the header
        process_list = {line.split("\t")[0] for line in lines if '__log:=' in line.split("\t")[1]}  # Extract PID if '__log:=' is in COMMAND
        for pid in process_list:
            try:
                subprocess.run(["kill", "-9", pid])
                print(f"Killed process with PID: {pid}")
            except Exception as e:
                print(f"Error killing process with PID {pid}. Error: {e}")
        print("In launch_stop, kill process complete")


        # Save current processes to a file
        after_file = '/home/tractor/processes_after_step3.txt'
        self.save_ps_to_file(after_file)

        # Compare before and after files and save the difference
        diff_file_counter = 1  # Initialize a counter for naming difference files
        while True:  # Find an unused filename
            diff_filename = f"/home/tractor/diff_{diff_file_counter}.txt"
            if not os.path.exists(diff_filename):
                break
            diff_file_counter += 1
        self.compare_and_save_diff('/home/tractor/processes_before.txt', after_file, diff_filename)                      

    def launch_location1(self):
        # copy from '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_location1.txt' 
        # to '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt'        
        # launch /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/launch/pure_pursuit_cub_cadet_oct23.launch
        # $ roslaunch ackermann_vehicle pure_pursuit_cub_cadet_oct23.launch origin_lat:=40.34534080 origin_lon:=-80.12894600 ref_lat:=40.34530756451623 ref_lon:=-80.12885480045905
        # launch /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/path_update_origin_lat_lon.py

        cmd = [
            "gnome-terminal",
            "--",
            "bash",
            "-c",
           # "source /opt/ros/noetic/setup.bash && "  # Source ROS setup.bash (adjust if needed)
           # "source /path/to/your/catkin_ws/devel/setup.bash && "  # Source your catkin workspace (adjust the path)
            "roslaunch ackermann_vehicle pure_pursuit_cub_cadet_oct23.launch "
            "origin_lat:=40.34534080 "
            "origin_lon:=-80.12894600 "
            "ref_lat:=40.34530756451623 "
            "ref_lon:=-80.12885480045905; "
            "exec bash"  # Keep the terminal open after the command finishes
        ]
        # Open a new terminal window and run the command
        subprocess.Popen(cmd)

        cmd = [
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "cd /home/tractor/bagfiles && "  # Change to the bagfiles directory
            "rosbag record -a; "  # Start recording all topics
            "exec bash"  # Keep the terminal open after the command finishes
            ]
        rosbag_process = subprocess.Popen(cmd)

    def launch_location2(self):
        print("In location2 Pausing 10 seconds")
        time.sleep(10)   
        cmd = [
            "gnome-terminal",
            "--",
            "bash",
            "-c",
           # "source /opt/ros/noetic/setup.bash && "  # Source ROS setup.bash (adjust if needed)
           # "source /path/to/your/catkin_ws/devel/setup.bash && "  # Source your catkin workspace (adjust the path)
            "roslaunch ackermann_vehicle pure_pursuit_cub_cadet_oct23.launch "
            "origin_lat:=40.48524688166667 "
            "origin_lon:=-80.332720941667 "
            "ref_lat:=40.48524688166667 "
            "ref_lon:=-80.332720941667; "
            "exec bash"  # Keep the terminal open after the command finishes
        ]

        print("In location2 , after cmd statement, Pausing 10 seconds")
        time.sleep(10)   
        # Open a new terminal window and run the command
        subprocess.Popen(cmd)
        print("In location2 , after Popen statement, Pausing 10 seconds")
        time.sleep(10)   
        cmd = [
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "cd /home/tractor/bagfiles && "  # Change to the bagfiles directory
            "rosbag record -a; "  # Start recording all topics
            "exec bash"  # Keep the terminal open after the command finishes
            ]
        print("In location2 , after 2nd cmd statement, Pausing 10 seconds")
        time.sleep(10)               
        rosbag_process = subprocess.Popen(cmd)
        print("In location2 , after rosbag Popen statement, Pausing 10 seconds")
        time.sleep(10)   
    def launch_location3(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location3.launch"])

    def launch_location4(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location4.launch"])
    def launch_drive_new_path(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "drive_new_path.launch"])        

    def gps_callback(self, data):
        self.last_gps_update = rospy.get_time()
        self.gps_status = data.status.status
        #print(f"In gps_callback, Status: {data.status.status}")
        # Update the GPS status icon
        self.update_gps_status_icon()

    def update_gps_status_icon(self):
        colors = {
            -1: "blue",
            0: "orange",
            1: "yellow",
            2: "green"
        }
        #print(f"Icon Color: {colors.get(self.gps_status, 'red')}")
        #print(f"GPS Status: {self.gps_status}")
        self.gps_status_canvas.config(bg=colors.get(self.gps_status, "red"))

    def check_gps_timeout(self):
        current_time = time.time()
        if current_time - self.last_gps_update > 1.0:
            self.gps_status = -2
            self.update_gps_status_icon()
        self.after(500, self.check_gps_timeout)  # Check again in 500ms        

if __name__ == "__main__":

    app = ROSLauncher()
    app.title("ROS Launcher GUI")
    app.mainloop()




