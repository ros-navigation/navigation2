import enum
import time
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

def create_heading_angle_list(minimal_set_trajectories: dict) -> list:
    heading_angles = set([angle for angle in minimal_set_trajectories.keys()])
    return sorted(
        list(heading_angles), key=lambda x: (x < 0, x)
    )

def read_config() -> dict:
    """
    Reads in the user defined parameters via JSON

    Returns
    -------
    dict
        Dictionary containing the user defined parameters
    """

    cur_dir = Path(__file__).parent
    config_path = cur_dir / "config.json"

    with open(config_path) as config_file:
        config = json.load(config_file)

    return config


def create_header(config: dict, minimal_set_trajectories: dict) -> dict:
    """
    Creates a dict containing all the fields to populate the header
    with for the output file

    Parameters
    ----------
    config: dict
        The dict containing user specified parameters
    minimal_set_trajectories: dict
        The minimal spanning set

    Returns
    -------
    dict
        A dictionary containing the fields to populate the header with
    """

    header_dict = {
        "version": VERSION,
        "date_generated": datetime.today().strftime("%Y-%m-%d"),
        "lattice_metadata": dict(),
        "primitives": [],
    }

    for key, value in config.items():
        header_dict["lattice_metadata"][key] = value

    header_dict["lattice_metadata"]["heading_angles"] = create_heading_angle_list(minimal_set_trajectories)

    return header_dict


def write_to_json(minimal_set_trajectories: dict, config: dict) -> None:
    """
    Writes the minimal spanning set to an output file

    Parameters
    ----------
    minimal_set_trajectories: dict
        The minimal spanning set
    config: dict
        The dict containing user specified parameters
    """

    output_dict = create_header(config, minimal_set_trajectories)

    trajectory_start_angles = list(minimal_set_trajectories.keys())

    heading_angle_list = create_heading_angle_list(minimal_set_trajectories)
    heading_lookup = {angle:idx for idx,angle in enumerate(heading_angle_list)}

    idx = 0
    for start_angle in sorted(trajectory_start_angles,
                              key=lambda x: (x < 0, x)):

        for trajectory in sorted(
            minimal_set_trajectories[start_angle],
            key=lambda x: x.parameters.end_angle
        ):

            traj_info = dict()
            traj_info["trajectory_id"] = idx
            traj_info["start_angle_index"] = heading_lookup[trajectory.parameters.start_angle]
            traj_info["end_angle_index"] = heading_lookup[trajectory.parameters.end_angle]
            traj_info["trajectory_radius"] = \
                trajectory.parameters.turning_radius
            traj_info["trajectory_length"] = round(
                trajectory.parameters.total_length, 5
            )
            traj_info["arc_length"] = round(
                trajectory.parameters.arc_length,
                5
            )
            traj_info["straight_length"] = round(
                trajectory.parameters.start_to_arc_distance
                + trajectory.parameters.arc_to_end_distance,
                5,
            )
            traj_info["poses"] = trajectory.path.to_output_format()

            output_dict["primitives"].append(traj_info)
            idx += 1

    output_dict["lattice_metadata"]["number_of_trajectories"] = idx

    output_path = Path(__file__).parent / config["output_file"]

    with open(output_path, "w") as output_file:
        json.dump(output_dict, output_file, indent="\t")


def save_visualizations(minimal_set_trajectories: dict) -> None:
    """
    Draws the visualizations for every trajectory from the minimal spanning set
    and saves it as an image

    Parameters
    ----------
    minimal_set_trajectories: dict
        The minimal spanning set
    """

    visualizations_folder = Path(__file__).parent / "visualizations"
    visualizations_folder.mkdir(exist_ok=True)

    for start_angle in minimal_set_trajectories.keys():

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, "b")

    plt.grid(True)
    plt.axis("square")
    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()

    output_path = visualizations_folder / "all_trajectories.png"
    plt.savefig(output_path)
    plt.clf()

    for start_angle in minimal_set_trajectories.keys():

        angle_in_deg = np.rad2deg(start_angle)

        if start_angle < 0 or start_angle > np.pi / 2:
            continue

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, "b")
            plt.xlim(left_x, right_x)
            plt.ylim(left_y, right_y)

        plt.grid(True)

        output_path = visualizations_folder / f"{angle_in_deg}.png"
        plt.savefig(output_path)
        plt.clf()


if __name__ == "__main__":
    config = read_config()

    start = time.time()
    lattice_gen = LatticeGenerator(config)
    minimal_set_trajectories = lattice_gen.run()
    print(f"Finished Generating. Took {time.time() - start} seconds")

    write_to_json(minimal_set_trajectories, config)
    save_visualizations(minimal_set_trajectories)
