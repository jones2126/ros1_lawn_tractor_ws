#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/gui_main_v3.py
import tkinter as tk
import tkinter.font as tkFont 
import subprocess

class ROSLauncher(tk.Tk):

    def __init__(self):
        super().__init__()

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
        self.create_button("Stop", self.launch_stop).grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 3", self.launch_location3).grid(row=2, column=1, sticky="nsew", padx=5, pady=5)
        self.create_button("Location 4", self.launch_location4).grid(row=3, column=1, sticky="nsew", padx=5, pady=5)

        # Configure rows and columns to expand
        for i in range(4):
            self.grid_rowconfigure(i, weight=1)

        for i in range(3):
            self.grid_columnconfigure(i, weight=1)

    
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
        subprocess.Popen(["roslaunch", "ackermann_vehicle", "cub_cadet_real_world_july23.launch"])

    def launch_location1(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location1.launch"])

    def launch_drive_new_path(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "drive_new_path.launch"])

    def launch_location2(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location2.launch"])

    def launch_stop(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "stop.launch"])

    def launch_location3(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location3.launch"])

    def launch_location4(self):
        subprocess.Popen(["roslaunch", "YOUR_PACKAGE_NAME", "location4.launch"])

if __name__ == "__main__":
    app = ROSLauncher()
    app.title("ROS Launcher GUI")
    app.mainloop()

