#!/usr/bin/env python3
'''
python3 read_comment_file.py
cd /home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing

'''

input_comment_file = "/home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/github_comment.txt"
with open(input_comment_file, 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]
    for line in content:
        results = line.split()
        github_comment = results[0]
        print(github_comment)
        print(type(github_comment))
input_comment_file = "/home/al/github_token.txt"
with open(input_comment_file, 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]
    for line in content:
        results = line.split()
        github_token = results[0]
        print(github_token)
        print(type(github_token))    