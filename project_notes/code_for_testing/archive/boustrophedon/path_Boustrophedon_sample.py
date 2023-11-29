#!/usr/bin/env python
'''
Script that shows a simple example of Boustrophedon coverage path planner

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_Boustrophedon_sample.py

'''

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def boustrophedon(env):
    path = []
    row_length = len(env[0])
    total_rows = len(env)
    
    for i in range(total_rows):
        if i % 2 == 0:  # Even rows: left to right
            for j in range(row_length):
                if env[i][j] == 0:
                    path.append((j, total_rows - i - 1))
        else:  # Odd rows: right to left
            for j in range(row_length - 1, -1, -1):
                if env[i][j] == 0:
                    path.append((j, total_rows - i - 1))
    return path


def visualize_path(environment, path):
    fig, ax = plt.subplots()
    total_rows = len(environment)
    row_length = len(environment[0])

    for i in range(total_rows):
        for j in range(row_length):
            if environment[i][j] == 1:
                ax.add_patch(patches.Rectangle((j, total_rows - i - 1), 1, 1, facecolor="black"))

    path_x = [x for x, y in path]
    path_y = [y for x, y in path]

    plt.plot(path_x, path_y, color="blue")
    plt.xlim(-1, row_length)
    plt.ylim(-1, total_rows)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


environment = [
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 0]
]

path = boustrophedon(environment)
visualize_path(environment, path)
