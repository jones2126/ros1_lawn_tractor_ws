#!/usr/bin/env python3  
#
# cd /home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/
# python3 github2.py


'''
Generate a new GitHub Token:

Go to GitHub and log in.
Navigate to Settings by clicking on your profile picture in the upper right corner.
In the left sidebar, click Developer settings.
Click Personal access tokens and then Tokens (classic).
Click Generate new token and provide the necessary permissions based on your needs.
Copy the generated token and store it securely.

'''
import os
import tkinter as tk
from tkinter import ttk, simpledialog
import subprocess  # Import the subprocess module
import time  # Import the time module

class Steps_to_Process(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        self.name = tk.StringVar()
        self.title_string = tk.StringVar()
        self.title_string.set("GitHub Update Tool")
        title_label = ttk.Label(self, textvariable=self.title_string, font=("TkDefaultFont", 12), wraplength=200)        

        step_1_button = ttk.Button(self, text="1. Edit comment", width=50, command=self.open_comment_file)
        step_2_button = ttk.Button(self, text="2. Push updates for ros1_lawn_tractor_ws", width=50, command=self.step_1_actions)        
        step_3_button = ttk.Button(self, text="placeholder", width=50, command=self.step_3_actions)
        step_4_button = ttk.Button(self, text="placeholder", width=50, command=self.step_4_actions)
        step_5_button = ttk.Button(self, text="placeholder", width=50, command=self.step_5_actions)
        step_6_button = ttk.Button(self, text="placeholder", width=50, command=self.step_6_actions)        

        self.columnconfigure(0, weight=1)             
        title_label.grid(row=0, column=0, columnspan=2)
        step_1_button.grid(row=1, column=0, sticky=tk.W)
        step_2_button.grid(row=2, column=0, sticky=tk.W)
        step_3_button.grid(row=3, column=0, sticky=tk.W)
        step_4_button.grid(row=4, column=0, sticky=tk.W)
        step_5_button.grid(row=5, column=0, sticky=tk.W)
        step_6_button.grid(row=6, column=0, sticky=tk.W)     

    def open_comment_file(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "cd ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/" + "\n"])
        time.sleep(1)   
        subprocess.check_output(["xdotool", "type", "nano github_comment.txt" + "\n"])
        time.sleep(1)

    def step_0_actions(self):
        working_directory1 = "--working-directory=/home/tractor"
        subprocess.call(["xdotool", "exec", "gnome-terminal", "--geometry=120x10+350+800", working_directory1])
        time.sleep(2)

    def step_1_actions(self):
        self.step_0_actions()
        input_comment_file = "/home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/github_comment.txt"
        with open(input_comment_file, 'r') as file:
            content = file.readlines()
            content = [x.strip() for x in content]
            for line in content:
                results = line.split()
                github_comment = results[0]

        input_token_file = "/home/tractor/Documents/github_token_validuntil_Mar_4_2024.txt"
        if os.path.exists(input_token_file):
            with open(input_token_file, 'r') as file:
                content = file.readlines()
                content = [x.strip() for x in content]
                for line in content:
                    results = line.split()
                    github_token = results[0]
        else:
            github_token = simpledialog.askstring("Input", "Enter your new GitHub token:", show='*')
            with open(input_token_file, 'w') as file:
                file.write(github_token)

        try:
            subprocess.check_output(["xdotool", "type", "cd ~/ros1_lawn_tractor_ws" + "\n"])
            time.sleep(1)
            subprocess.check_output(["xdotool", "type", "git add ." + "\n"])
            time.sleep(1)
            github_cmd = "git commit -m "
            subprocess.check_output(["xdotool", "type", github_cmd + '"' + github_comment + '"' + "\n"])
            time.sleep(1)
            subprocess.check_output(["xdotool", "type", "git push origin master" + "\n"])
            time.sleep(10)
            subprocess.check_output(["xdotool", "type", "jones2126" + "\n"])
            time.sleep(3)
            subprocess.check_output(["xdotool", "type", github_token + "\n"])
            time.sleep(1)
        except subprocess.CalledProcessError:
            github_token = simpledialog.askstring("Input", "Authentication failed. Enter your new GitHub token:", show='*')
            with open(input_token_file, 'w') as file:
                file.write(github_token)
            subprocess.check_output(["xdotool", "type", github_token + "\n"])
            time.sleep(1)

    def step_2_actions(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "history" + "\n"])
        time.sleep(1) 

    def step_3_actions(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "history" + "\n"])
        time.sleep(1) 

    def step_4_actions(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "history" + "\n"])
        time.sleep(1)

    def step_5_actions(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "history" + "\n"])
        time.sleep(1)

    def step_6_actions(self):
        self.step_0_actions()
        subprocess.check_output(["xdotool", "type", "history" + "\n"])
        time.sleep(1)

class ROS_GUI(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.title("GitHub Update Tool")
        self.geometry("200x300")
        self.resizable(width=False, height=False)

        Steps_to_Process(self).grid(sticky=(tk.E + tk.W + tk.N + tk.S))
        self.columnconfigure(0, weight=1)

if __name__ == '__main__':
    app = ROS_GUI()
    app.mainloop()
