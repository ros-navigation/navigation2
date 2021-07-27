import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def plot_arrow(x, y, yaw, length=1.0, fc="r", ec="k"):
    """
    Plot arrow
    """
    plt.arrow(x, y, length * np.cos(yaw), length *
              np.sin(yaw), width=0.05*length, length_includes_head=True)
    plt.plot(x, y)
    plt.plot(0, 0)


def read_trajectories_data(file_path):

    with open(file_path) as data_file:
        trajectory_data = json.load(data_file)

    return trajectory_data


cur_file_path = Path(__file__)
trajectory_file_path = cur_file_path.parent.parent / "output.json"


trajectory_data = read_trajectories_data(trajectory_file_path)

for primitive in trajectory_data["primitives"]:
    arrow_length = (primitive["arc_length"] +
                    primitive["straight_length"]) / len(primitive["poses"])

    if arrow_length == 0:
        arrow_length = 1

    xs = [pose[0] for pose in primitive["poses"]]
    ys = [pose[1] for pose in primitive["poses"]]

    for x, y, yaw in primitive["poses"]:
        plot_arrow(x, y, yaw, length=arrow_length)

    plt.scatter(xs, ys)
    plt.grid(True)
    plt.axis('square')

    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()
    plt.xlim(left_x - 1.5*arrow_length, right_x + 1.5*arrow_length)
    plt.ylim(left_y - 1.5*arrow_length, right_y + 1.5*arrow_length)

    plt.title(f'Trajectory ID: {primitive["trajectory_id"]}')
    plt.figtext(
        0.7, 0.9, f'Start: {np.rad2deg(primitive["start_angle"])}\nEnd: {np.rad2deg(primitive["end_angle"])}')

    plt.show()
