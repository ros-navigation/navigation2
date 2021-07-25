import json
import logging
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

from lattice_generator import LatticeGenerator

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

VERSION = 1.0


def read_config():
    with open("config.json") as config_file:
        config = json.load(config_file)

    return config


def create_header(config, minimal_set_trajectories):
    header_dict = {"version": VERSION, "dateGenerated": datetime.today().strftime(
        '%Y-%m-%d'), "latticeMetadata": dict(), "primitives": []}

    for key, value in config.items():
        header_dict["latticeMetadata"][key] = value

    heading_angles = set([angle for angle in minimal_set_trajectories.keys()])
    header_dict["latticeMetadata"]["headingAngles"] = sorted(
        list(heading_angles))

    return header_dict


def write_to_json(minimal_set_trajectories, config):
    output_dict = create_header(config, minimal_set_trajectories)

    idx = 0
    for start_angle in minimal_set_trajectories.keys():

        for trajectory in sorted(minimal_set_trajectories[start_angle], key=lambda x: x.parameters.end_angle):

            traj_info = dict()
            traj_info['trajectoryId'] = idx
            traj_info['startAngle'] = trajectory.parameters.start_angle
            traj_info['endAngle'] = trajectory.parameters.end_angle
            traj_info['radius'] = trajectory.parameters.radius
            traj_info['trajectoryLength'] = round(
                trajectory.parameters.total_length, 5)
            traj_info['arcLength'] = round(trajectory.parameters.arc_length, 5)
            traj_info['straightLength'] = round(trajectory.parameters.start_to_arc_distance +
                                                trajectory.parameters.arc_to_end_distance, 5)
            traj_info['poses'] = trajectory.path.to_output_format()

            output_dict["primitives"].append(traj_info)
            idx += 1

    output_dict["latticeMetadata"]["numberOfTrajectories"] = idx

    with open(config['outputFile'], 'w') as output_file:
        json.dump(output_dict, output_file, indent="\t")


def save_visualizations(minimal_set_trajectories):

    for start_angle in minimal_set_trajectories.keys():

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, "b")

    plt.grid(True)
    plt.axis('square')
    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()
    plt.savefig("visualizations/all_trajectories.png")
    plt.clf()

    for start_angle in minimal_set_trajectories.keys():

        angle_in_deg = np.rad2deg(start_angle)

        if start_angle < 0 or start_angle > np.pi/2:
            continue

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, "b")
            plt.xlim(left_x, right_x)
            plt.ylim(left_y, right_y)

        plt.grid(True)
        plt.savefig(f'visualizations/{angle_in_deg}.png')
        plt.clf()


if __name__ == "__main__":
    config = read_config()

    lattice_gen = LatticeGenerator(config)
    minimal_set_trajectories = lattice_gen.run()

    write_to_json(minimal_set_trajectories, config)
    save_visualizations(minimal_set_trajectories)
