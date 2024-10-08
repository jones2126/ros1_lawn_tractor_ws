#!/usr/bin/env python3  
'''
program to launch ros commands so you don't have to remember or type the commands

'''
import tkinter as tk
from tkinter import ttk
import subprocess
import time


class Steps_to_Process(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        self.name = tk.StringVar()
        self.title_string = tk.StringVar()
        self.title_string.set("Lawn Tractor Startup")
        title_label = ttk.Label(self, textvariable=self.title_string, font=("TkDefaultFont", 12), wraplength=200)        

        step_1_button = ttk.Button(self, text="1. Gazebo", width=50, command=self.step_1_actions)
        step_2_button = ttk.Button(self, text="2. teb_planner", width=50, command=self.step_2_actions)
        step_3_button = ttk.Button(self, text="3. RVIZ @ 435", width=50, command=self.step_3_actions)
        step_4_button = ttk.Button(self, text="rostopic echo /move_base_simple/goal", width=50, command=self.step_4_actions)
        step_5_button = ttk.Button(self, text="rostopic echo /cmd_vel", width=50, command=self.step_5_actions)
        step_6_button = ttk.Button(self, text="rosbag record -a", width=50, command=self.step_6_actions)
        step_7_button = ttk.Button(self, text="3. RVIZ @ 62 Collins", width=50, command=self.step_7_actions)
        step_8_button = ttk.Button(self, text="tbd", width=50, command=self.step_8_actions)
        step_9_button = ttk.Button(self, text="Run mmbf mission", width=50, command=self.step_9_actions)        

        # Layout form
        self.columnconfigure(0, weight=1)             
        title_label.grid(row=0, column=0, columnspan=2)
        step_1_button.grid(row=1, column=0, sticky=tk.W)
        step_2_button.grid(row=2, column=0, sticky=tk.W)
        step_3_button.grid(row=3, column=0, sticky=tk.W)
        step_4_button.grid(row=4, column=0, sticky=tk.W)
        step_5_button.grid(row=5, column=0, sticky=tk.W)
        step_6_button.grid(row=6, column=0, sticky=tk.W)
        step_7_button.grid(row=7, column=0, sticky=tk.W)
        step_8_button.grid(row=8, column=0, sticky=tk.W)
        step_9_button.grid(row=9, column=0, sticky=tk.W)   

    def RPi_login_steps(self):  # unused currently.  Leaving in case I need it again.
        time.sleep(2)
        cmd_RPi_login = "plink RPI_local_mofi_6c -pw ubuntu"  # command to login to RPi
        #cmd_RPi_login = "plink tractor -pw ubuntu"  # command to login to RPi
        time.sleep(.1)
        subprocess.check_output(["xdotool", "type", cmd_RPi_login + "\n"])
        time.sleep(5) # delay for the login process to the RPi
        subprocess.check_output(["xdotool", "type", "\n"])
        time.sleep(.5)

    def step_0_actions(self): # used for testing
        working_directory1 = "--working-directory=/home/al"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+350+800", working_directory1])
        time.sleep(2) # delay for sensors to fire up
        #subprocess.check_output(["xdotool", "type", "cd ~/ros1_lawn_tractor_ws" + "\n"])
        #time.sleep(3) # delay for testing
        #subprocess.check_output(["xdotool", "type", "source devel/setup.bash" + "\n"])
        #time.sleep(3) # delay for testing

    def step_1_actions(self): # Gazebo
        self.step_0_actions()
        time.sleep(1)        
        subprocess.check_output(["xdotool", "type", "roslaunch ackermann_vehicle cub_cadet_gazebo.launch" + "\n"])
        time.sleep(1) 
   
    def step_2_actions(self):   # movebase
        self.step_0_actions()
        time.sleep(2)
        subprocess.check_output(["xdotool", "type", "roslaunch ackermann_vehicle mbf_newteb.launch" + "\n"])
        time.sleep(1) 
   
    def step_3_actions(self):   # rviz       
        self.step_0_actions()
        time.sleep(2)        
        subprocess.check_output(["xdotool", "type", "rviz -d ~/ros1_lawn_tractor_ws/src/ackermann_vehicle/maps/tractor.rviz" + "\n"])
        time.sleep(1) 
             
    def step_4_actions(self):  # rostopic echo /move_base_simple/goal
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "rostopic echo /move_base_simple/goal" + "\n"])
        time.sleep(1)

    def step_5_actions(self): # rostopic echo /cmd_vel
        self.step_0_actions()
        time.sleep(3) # delay 
        subprocess.check_output(["xdotool", "type", "rostopic echo /cmd_vel" + "\n"])
        time.sleep(1)
 
    def step_6_actions(self):  # rosbag
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "cd ~/.ros/bagfiles" + "\n"])
        time.sleep(1)
        subprocess.check_output(["xdotool", "type", "rosbag record -a" + "\n"])
        time.sleep(1)
        
    def step_7_actions(self):   # open
        self.step_0_actions()
        time.sleep(2)        
        subprocess.check_output(["xdotool", "type", "rviz -d /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/maps/map_testing.rviz" + "\n"])
        time.sleep(1) 
   
    def step_8_actions(self):   # open      
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "cd ~/ros1_lawn_tractor_ws" + "\n"])
        time.sleep(1)         

    def step_9_actions(self):   # running a set of waypoints       
        self.step_0_actions()
        time.sleep(5) # delay because my computer is slow with Gazebo running         
        subprocess.check_output(["xdotool", "type", "rosrun ackermann_vehicle mbf_exec_mission.py" + "\n"])
        time.sleep(1)      


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