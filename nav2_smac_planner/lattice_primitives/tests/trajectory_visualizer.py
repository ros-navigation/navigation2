# Copyright (c) 2021, Matthew Booker
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. Reserved.

"""
This script is used visualize each trajectory individually.

This helps to better understand how a single trajectory looks and
to ensure that the x, y, and yaw values are correct. This is mainly
used for debugging when making changes to parts of the code.
However, if you would like to see how each trajectory in your
ouput file looks then you can run this script.
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt

import numpy as np


def plot_arrow(x, y, yaw, length=1.0, fc='r', ec='k'):
    """Plot arrow."""
    plt.arrow(
        x,
        y,
        length * np.cos(yaw),
        length * np.sin(yaw),
        width=0.05 * length,
        length_includes_head=True,
    )
    plt.plot(x, y)
    plt.plot(0, 0)


def read_trajectories_data(file_path):

    with open(file_path) as data_file:
        trajectory_data = json.load(data_file)

    return trajectory_data


cur_file_path = Path(__file__)
trajectory_file_path = cur_file_path.parent.parent / 'output.json'


trajectory_data = read_trajectories_data(trajectory_file_path)
min_x = min(
    [
        min([pose[0] for pose in primitive['poses']])
        for primitive in trajectory_data['primitives']
    ]
)
max_x = max(
    [
        max([pose[0] for pose in primitive['poses']])
        for primitive in trajectory_data['primitives']
    ]
)

min_y = min(
    [
        min([pose[1] for pose in primitive['poses']])
        for primitive in trajectory_data['primitives']
    ]
)
max_y = max(
    [
        max([pose[1] for pose in primitive['poses']])
        for primitive in trajectory_data['primitives']
    ]
)

heading_angles = trajectory_data['lattice_metadata']['heading_angles']

for primitive in trajectory_data['primitives']:
    arrow_length = (primitive['arc_length'] + primitive['straight_length']) / len(
        primitive['poses']
    )

    if arrow_length == 0:
        arrow_length = max_x / len(primitive['poses'])

    xs = np.array([pose[0] for pose in primitive['poses']])
    ys = np.array([pose[1] for pose in primitive['poses']])

    lengths = np.sqrt((xs[1:] - xs[:-1]) ** 2 + (ys[1:] - ys[:-1]) ** 2)
    print('Distances between points: ', lengths)

    for x, y, yaw in primitive['poses']:
        plot_arrow(x, y, yaw, length=arrow_length)

    plt.scatter(xs, ys)
    plt.grid(True)
    plt.axis('square')

    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()
    plt.xlim(1.2 * min_x, 1.2 * max_x)
    plt.ylim(1.2 * min_y, 1.2 * max_y)

    start_angle = np.rad2deg(heading_angles[primitive['start_angle_index']])
    end_angle = np.rad2deg(heading_angles[primitive['end_angle_index']])

    plt.title(f"Trajectory ID: {primitive['trajectory_id']}")
    plt.figtext(0.7, 0.9, f'Start: {start_angle}\nEnd: {end_angle}')

    plt.show()
