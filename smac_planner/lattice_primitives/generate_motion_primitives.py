import json
import logging
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from lattice_generator import LatticeGenerator

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

VERSION = 1.0


def read_config():

    cur_dir = Path(__file__).parent
    config_path = cur_dir / "config.json"

    with open(config_path) as config_file:
        config = json.load(config_file)

    return config


def create_header(config, minimal_set_trajectories):
    header_dict = {"version": VERSION, "date_generated": datetime.today().strftime(
        '%Y-%m-%d'), "lattice_metadata": dict(), "primitives": []}

    for key, value in config.items():
        header_dict["lattice_metadata"][key] = value

    heading_angles = set([angle for angle in minimal_set_trajectories.keys()])
    header_dict["lattice_metadata"]["heading_angles"] = sorted(
        list(heading_angles), key=lambda x: (x < 0, x))

    return header_dict


def write_to_json(minimal_set_trajectories, config):
    output_dict = create_header(config, minimal_set_trajectories)

    trajectory_start_angles = list(minimal_set_trajectories.keys())

    idx = 0
    for start_angle in sorted(trajectory_start_angles, key=lambda x: (x < 0, x)):

        for trajectory in sorted(minimal_set_trajectories[start_angle], key=lambda x: x.parameters.end_angle):

            traj_info = dict()
            traj_info['trajectory_id'] = idx
            traj_info['start_angle'] = trajectory.parameters.start_angle
            traj_info['end_angle'] = trajectory.parameters.end_angle
            traj_info['trajectory_radius'] = trajectory.parameters.turning_radius
            traj_info['trajectory_length'] = round(
                trajectory.parameters.total_length, 5)
            traj_info['arc_length'] = round(
                trajectory.parameters.arc_length, 5)
            traj_info['straight_length'] = round(trajectory.parameters.start_to_arc_distance +
                                                 trajectory.parameters.arc_to_end_distance, 5)
            traj_info['poses'] = trajectory.path.to_output_format()

            output_dict["primitives"].append(traj_info)
            idx += 1

    output_dict["lattice_metadata"]["number_of_trajectories"] = idx

    output_path = Path(__file__).parent / config["output_file"]

    with open(output_path, 'w') as output_file:
        json.dump(output_dict, output_file, indent="\t")


def save_visualizations(minimal_set_trajectories):

    visualizations_folder = Path(__file__).parent / "visualizations"

    for start_angle in minimal_set_trajectories.keys():

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, "b")

    plt.grid(True)
    plt.axis('square')
    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()

    output_path = visualizations_folder / "all_trajectories.png"
    plt.savefig(output_path)
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

        output_path = visualizations_folder / f'{angle_in_deg}.png'
        plt.savefig(output_path)
        plt.clf()


if __name__ == "__main__":
    config = read_config()

    lattice_gen = LatticeGenerator(config)
    minimal_set_trajectories = lattice_gen.run()

    write_to_json(minimal_set_trajectories, config)
    save_visualizations(minimal_set_trajectories)
