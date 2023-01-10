#!/usr/bin/env python3  
#

import tkinter as tk
from tkinter import ttk
import subprocess
import time
'''
issues: 
- speed up the login process by looking for the $ which shows the login has completed
- add steps for cmd_vel
- add display for heading


'''

class Steps_to_Process(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        self.name = tk.StringVar()
        self.title_string = tk.StringVar()
        self.title_string.set("Lawn Tractor Startup")
        title_label = ttk.Label(self, textvariable=self.title_string, font=("TkDefaultFont", 12), wraplength=200)        

        step_1_button = ttk.Button(self, text="Gazebo", width=50, command=self.step_1_actions)
        step_2_button = ttk.Button(self, text="nav_newteb", width=50, command=self.step_2_actions)
        step_3_button = ttk.Button(self, text="RVIZ", width=50, command=self.step_3_actions)
        step_4_button = ttk.Button(self, text="GPS ODOM", width=50, command=self.step_4_actions)
        step_5_button = ttk.Button(self, text="Move Base Launch", width=50, command=self.step_5_actions)
        step_6_button = ttk.Button(self, text="Save Rosbag", width=50, command=self.step_6_actions)        


        # Layout form
        self.columnconfigure(0, weight=1)             
        title_label.grid(row=0, column=0, columnspan=2)
        step_1_button.grid(row=1, column=0, sticky=tk.W)
        step_2_button.grid(row=2, column=0, sticky=tk.W)
        step_3_button.grid(row=3, column=0, sticky=tk.W)
        step_4_button.grid(row=4, column=0, sticky=tk.W)
        step_5_button.grid(row=5, column=0, sticky=tk.W)
        step_6_button.grid(row=6, column=0, sticky=tk.W)     

    def RPi_login_steps(self):
        time.sleep(2)
        cmd_RPi_login = "plink RPI_local_mofi_6c -pw ubuntu"  # command to login to RPi
        #cmd_RPi_login = "plink tractor -pw ubuntu"  # command to login to RPi
        time.sleep(.1)
        subprocess.check_output(["xdotool", "type", cmd_RPi_login + "\n"])
        time.sleep(5) # delay for the login process to the RPi
        subprocess.check_output(["xdotool", "type", "\n"])
        time.sleep(.5)

    def step_0_actions(self): # main launch of sensors
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+350+800", working_directory1])
        time.sleep(2) # delay for sensors to fire up
        #subprocess.check_output(["xdotool", "type", "cd ~/ros1_lawn_tractor_ws" + "\n"])
        #time.sleep(3) # delay for testing
        #subprocess.check_output(["xdotool", "type", "source devel/setup.bash" + "\n"])
        #time.sleep(3) # delay for testing

    def step_1_actions(self): # main launch of sensors
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "roslaunch ackermann_vehicle ack_new.launch" + "\n"])
        time.sleep(1) # delay for testing
   
    def step_2_actions(self):   # movebase
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "roslaunch ackermann_vehicle nav_newteb.launch" + "\n"])
        time.sleep(1) # delay for testing
   
    def step_3_actions(self):   # rviz       
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+550+800", working_directory1])
        time.sleep(3)
        subprocess.check_output(["xdotool", "type", "rviz -d ~/ros1_lawn_tractor_ws/src/ackermann_vehicle/maps/435_tractor.rviz" + "\n"])
             
    def step_4_actions(self):  # gps odom
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+750+800", working_directory1])
        self.RPi_login_steps()
        subprocess.check_output(["xdotool", "type", "cd ~/catkin_ws" + "\n"])
        time.sleep(.5)
        subprocess.check_output(["xdotool", "type", "rosrun beginner_tutorials gps_odom.py" + "\n"])
    def step_5_actions(self): # move base
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+950+700", working_directory1])
        self.RPi_login_steps()
        subprocess.check_output(["xdotool", "type", "cd ~/catkin_ws" + "\n"])
        time.sleep(.5)
        subprocess.check_output(["xdotool", "type", "roslaunch lawn_tractor_sim lawn_tractor.launch" + "\n"])   
    def step_6_actions(self):  # rosbag
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+1150+700", working_directory1])
        self.RPi_login_steps()
        subprocess.check_output(["xdotool", "type", "bash ros_bagfile.sh" + "\n"])  


class ROS_GUI(tk.Tk):
    """ROS GUI Main Application"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # set the window properties
        self.title("ROS GUI")
        self.geometry("200x300")
        self.resizable(width=False, height=False)

        # Define the UI
        Steps_to_Process(self).grid(sticky=(tk.E + tk.W + tk.N + tk.S))
        self.columnconfigure(0, weight=1)

if __name__ == '__main__':
    app = ROS_GUI()
    app.mainloop()