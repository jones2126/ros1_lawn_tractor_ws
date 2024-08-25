#!/usr/bin/env python3
import tkinter as tk
import tkinter.font as tkFont 
import subprocess
import rospy
import socket
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import time
import signal
import re
import os
from sensor_msgs.msg import Imu     # to get IMU data
import tf.transformations           # to get IMU data
import math                         # to get IMU data

class ROSLauncher(tk.Tk):

    def __init__(self):
        super().__init__()
        self.process = None
        self.last_gps_update = 0
        self.ros_initialized = False

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

        # Frame for array data
        self.array_frame = tk.Frame(self)
        self.array_frame.grid(row=3, column=1, columnspan=2, sticky="se", padx=10, pady=10)  # Changed from column=2 to column=1, columnspan=2

        # Create a frame for status indicators
        self.status_frame = tk.Frame(self)
        self.status_frame.grid(row=4, column=0, columnspan=3, sticky="se", padx=10, pady=10)


        # Configure columns in array_frame
        self.array_frame.columnconfigure(0, weight=1)
        self.array_frame.columnconfigure(1, weight=1)

        # Labels for array data
        tk.Label(self.array_frame, text="Steering Pot:", font=("Arial", 14)).grid(row=0, column=0, sticky="e", padx=(0, 5))
        self.filtered_steer_pot_int_label = tk.Label(self.array_frame, text="N/A", font=("Arial", 14))
        self.filtered_steer_pot_int_label.grid(row=0, column=1, sticky="w")

        tk.Label(self.array_frame, text="Steer Set Point:", font=("Arial", 14)).grid(row=1, column=0, sticky="e", padx=(0, 5))
        self.steer_set_point_label = tk.Label(self.array_frame, text="N/A", font=("Arial", 14))
        self.steer_set_point_label.grid(row=1, column=1, sticky="w")

        tk.Label(self.array_frame, text="Steer Effort:", font=("Arial", 14)).grid(row=2, column=0, sticky="e", padx=(0, 5))
        self.steer_effort_label = tk.Label(self.array_frame, text="N/A", font=("Arial", 14))
        self.steer_effort_label.grid(row=2, column=1, sticky="w")

        tk.Label(self.array_frame, text="Heading:", font=("Arial", 14)).grid(row=3, column=0, sticky="e", padx=(0, 5))
        self.heading_label = tk.Label(self.array_frame, text="N/A", font=("Arial", 14))
        self.heading_label.grid(row=3, column=1, sticky="w")      

        # GPS status
        tk.Label(self.status_frame, text="GPS", font=("Arial", 10)).grid(row=0, column=0, padx=5)
        self.gps_status_canvas = tk.Canvas(self.status_frame, width=30, height=30, bg="red", highlightthickness=0)
        self.gps_status_canvas.grid(row=1, column=0, padx=5)

        # Serial1 status
        tk.Label(self.status_frame, text="Serial1", font=("Arial", 10)).grid(row=0, column=1, padx=5)
        self.ser1_status_canvas = tk.Canvas(self.status_frame, width=30, height=30, bg="red", highlightthickness=0)
        self.ser1_status_canvas.grid(row=1, column=1, padx=5)

        # Serial2 status
        tk.Label(self.status_frame, text="Serial2", font=("Arial", 10)).grid(row=0, column=2, padx=5)
        self.ser2_status_canvas = tk.Canvas(self.status_frame, width=30, height=30, bg="red", highlightthickness=0)
        self.ser2_status_canvas.grid(row=1, column=2, padx=5)

        # Configure rows and columns to expand
        for i in range(5):  # Changed from 4 to 5 to accommodate the new row
            self.grid_rowconfigure(i, weight=1)

        for i in range(3):
            self.grid_columnconfigure(i, weight=1)

        # Start the periodic checks
        self.periodic_check_ros()
        self.check_gps_timeout()

    def log_message(self, message, level="info"):
        if self.ros_initialized:
            if level == "info":
                rospy.loginfo(message)
            elif level == "warn":
                rospy.logwarn(message)
            elif level == "error":
                rospy.logerr(message)
        else:
            print(f"[{level.upper()}] {message}")

    def check_ros_master(self):
        try:
            s = socket.create_connection(("localhost", 11311), timeout=1)
            s.close()
            return True
        except socket.error:
            return False

    def initialize_ros(self):
        if not self.ros_initialized:
            try:
                rospy.init_node('gui_listener', anonymous=True)
                self.fix_subscriber = rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
                self.serial_status_subscriber = rospy.Subscriber("/serial_status", Float64MultiArray, self.serial_status_callback)
                self.array_subscriber = rospy.Subscriber("/my_array_topic", Float64MultiArray, self.array_callback)
                self.imu_subscriber = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
                self.ros_initialized = True
                self.log_message("ROS node initialized and topics subscribed")
            except rospy.ROSException as e:
                self.log_message(f"Failed to initialize ROS: {e}", "error")
                self.ros_initialized = False

    def extract_pid_command_from_ps_file(self, filename):
        with open(filename, 'r') as file:
            lines = file.readlines()[1:]  # Exclude header
            return {(line.split()[1], line.split()[-1]) for line in lines}

    def compare_and_save_diff(self, before_file, after_file, diff_file):
        before_set = self.extract_pid_command_from_ps_file(before_file)
        after_set = self.extract_pid_command_from_ps_file(after_file)
        diff = after_set - before_set
        
        with open(diff_file, 'w') as file:
            file.write("PID\tCOMMAND\n")
            for pid, cmd in diff:
                file.write(f"{pid}\t{cmd}\n")                    

    def periodic_check_ros(self):
        if self.check_ros_master():
            if not self.ros_initialized:
                self.initialize_ros()
        else:
            if self.ros_initialized:
                self.log_message("ROS master not found. Resetting ROS initialization.", "warn")
                self.ros_initialized = False
                if self.fix_subscriber:
                    self.fix_subscriber.unregister()
                if self.serial_status_subscriber:
                    self.serial_status_subscriber.unregister()


                if self.array_subscriber:  # Add this line
                    self.array_subscriber.unregister()  # Add this line
                self.fix_subscriber = None
                self.serial_status_subscriber = None
                self.array_subscriber = None  # Add this line
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
        with open(filename, 'w') as file:
            subprocess.run(["ps", "aux"], stdout=file)
            print(f"In save_ps_to_file, Results saved to {filename}")

    def extract_pid_command_from_ps_file(self, filename):
        with open(filename, 'r') as file:
            lines = file.readlines()[1:]    # Exclude header from the file
            pid_command_set = set()         # Initialize an empty set
            for line in lines:              # Iterate over each line and add the PID and COMMAND pairs to the set
                split_line = re.split(r'\s+', line.strip())
                pid = split_line[1]
                command = " ".join(split_line[10:])
                pid_command_set.add((pid, command))
            return pid_command_set                  

    def log_command(self, command):
        # this is a future function to make a log entry in a history file (i.e. .xlsx file)
        # I have not implemented this yet, but wanted to save this starting code.
        # These statements would need to be added to the 'launch_start' function
        #       command = "roslaunch ackermann_vehicle cub_cadet_real_world_oct23.launch"
        #       self.process = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])
        #       self.log_command(command)
        # This statement, 'self.excel_file_path = os.path.expanduser('~/command_line_history.xlsx')'
        # would need to be added to 'def __init__(self):'
        epoch_time = int(time.time())
        human_readable_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Read existing Excel file if it exists, otherwise create a new DataFrame
        if os.path.exists(self.excel_file_path):
            df = pd.read_excel(self.excel_file_path)
        else:
            df = pd.DataFrame(columns=['Epoch Time', 'Human Readable Time', 'Command'])
        
        # Append new command
        new_row = pd.DataFrame({
            'Epoch Time': [epoch_time],
            'Human Readable Time': [human_readable_time],
            'Command': [command]
        })
        df = pd.concat([df, new_row], ignore_index=True)
        
        # Save updated DataFrame to Excel file
        df.to_excel(self.excel_file_path, index=False)
        
        print(f"Logged command: {command}")
   
    def launch_start(self):
        pass

    def launch_stop(self):
        print("In launch_stop")
        try:
            print("Stopping rosbag recording")
            output = subprocess.check_output(["pgrep", "-f", "rosbag record -a"]).decode().strip()
            if output:
                pids = output.split()
                for pid in pids:
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
                    subprocess.run(["kill", "-9", pid])
                    print(f"Stopped rosbag recording process with PID: {pid}")
        except Exception as e:
            print(f"Error while trying to stop rosbag recording: {e}")
        time.sleep(2)
        self.fix_subscriber.unregister()
        self.serial_status_subscriber.unregister()

        if hasattr(self, 'imu_subscriber') and self.imu_subscriber:
            self.imu_subscriber.unregister()
            print("Unregistered the IMU subscriber")

        print("Unregistered the /fix, /serial_status and /imu subscribers, Pausing 5 seconds")
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

    def launch_location1(self):
        # Your existing implementation
        pass

    def launch_location2(self):
        self.log_message("Starting launch_location2 - saving list of processes running")
        self.save_ps_to_file("processes_before.txt")  # Save initial processes       
        self.log_message("In launch_location2 Pausing 2 seconds")
        time.sleep(2)   
        cmd = [
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "roslaunch ackermann_vehicle cub_cadet_location_2.launch; "
            "exec bash"  # Keep the terminal open after the command finishes
        ]
        self.log_message("In launch_location2, initiating launch file")
        time.sleep(1)   
        # Open a new terminal window and run the command
        subprocess.Popen(cmd)
        self.log_message("In launch_location2, after Popen statement, Pausing 15 seconds")
        time.sleep(15)   
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
        self.log_message("At the end of launch_location2")

    def launch_location3(self):
        pass

    def launch_location4(self):
        pass

    def launch_drive_new_path(self):
        pass

    def array_callback(self, msg):
        if len(msg.data) > 15:  # Assuming steer_effort is the 16th element
            filtered_steer_pot_int = msg.data[2]
            steer_set_point = msg.data[4]
            steer_effort = msg.data[15]
            
            self.filtered_steer_pot_int_label.config(text=f"{filtered_steer_pot_int:.2f}")
            self.steer_set_point_label.config(text=f"{steer_set_point:.2f}")
            self.steer_effort_label.config(text=f"{steer_effort:.2f}")

    def gps_callback(self, msg):
        self.last_gps_update = rospy.get_time()
        self.update_gps_status_icon(msg.status.status)

    def serial_status_callback(self, data):
        if len(data.data) >= 2:
            self.update_serial_status_icon(self.ser1_status_canvas, data.data[0] > 0.5)
            self.update_serial_status_icon(self.ser2_status_canvas, data.data[1] > 0.5)

    def imu_callback(self, msg):
        # Extract the orientation quaternion
        orientation_q = msg.orientation
        # Convert quaternion to Euler angles
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # Convert yaw to degrees
        heading_deg = math.degrees(yaw)
        # Ensure heading is between -180 and 180 degrees (to match your IMU code)
        if heading_deg > 180:
            heading_deg -= 360
        elif heading_deg < -180:
            heading_deg += 360
        # Update the heading label
        self.heading_label.config(text=f"{heading_deg:.2f}°")

    def update_serial_status_icon(self, canvas, status):
        color = "green" if status else "red"
        canvas.config(bg=color)

    def update_gps_status_icon(self, status):
        colors = {
            -1: "blue",
            0: "orange",
            1: "yellow",
            2: "green"
        }
        self.gps_status_canvas.config(bg=colors.get(status, "red"))

    def check_gps_timeout(self):
        current_time = time.time()
        if current_time - self.last_gps_update > 1.0:
            self.update_gps_status_icon(-2)  # Use -2 to indicate timeout
        self.after(500, self.check_gps_timeout)

if __name__ == "__main__":
    app = ROSLauncher()
    app.title("ROS Launcher GUI")
    app.mainloop()        