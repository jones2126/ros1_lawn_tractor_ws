#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/gui_main_v3.py
import tkinter as tk
import tkinter.font as tkFont 
import subprocess
import rospy
import socket
from sensor_msgs.msg import NavSatFix
import time
import signal

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
        self.periodic_check_ros()
        self.check_gps_timeout()

    def check_ros_master(self):
        try:
            s = socket.create_connection(("localhost", 11311), timeout=1)
            s.close()
            return True
        except socket.error:
            return False        

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

    # Sample launch functions for each button

    def launch_start(self):
        self.process = subprocess.Popen(["roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])

    def launch_location1(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location1.launch"])

    def launch_drive_new_path(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "drive_new_path.launch"])

    def launch_location2(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location2.launch"])

    def launch_stop_old(self):
        # Unregister the /fix subscriber
        self.fix_subscriber.unregister()
        print("Unregistered the /fix subscriber")
        time.sleep(2)

        # Execute the rosnode cleanup before terminating the ROS process
        subprocess.run(['rosnode', 'cleanup'])
        print("Executed rosnode cleanup")
        time.sleep(2)
        # If there's an active process, terminate it
        if self.process:
            self.process.terminate()
            self.process = None
            print("Terminated the ROS process")
        
        # Adding a delay to ensure processes have time to shut down
        time.sleep(5)



    def launch_stop(self):
        # 1. Unregister the /fix subscriber
        self.fix_subscriber.unregister()
        print("Unregistered the /fix subscriber")

        rospy.signal_shutdown("GUI Shutdown")
        time.sleep(2)

        
        # 2. Signal ROS nodes for a graceful shutdown
        if self.process:
            self.process.send_signal(signal.SIGINT)
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
        if self.process:
            self.process.terminate()
            self.process = None
            print("Terminated the ROS process")
        
        # 6. Final delay for cleanup
        time.sleep(5)


    def launch_location3(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location3.launch"])

    def launch_location4(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location4.launch"])

    def gps_callback(self, data):
        self.last_gps_update = rospy.get_time()
        self.gps_status = data.status.status
        print(f"In gps_callback, Status: {data.status.status}")
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




