from lattice_generator import LatticeGenerator

import matplotlib.pyplot as plt
import json

from datetime import datetime

VERSION = 1.0

def read_config():
    with open("config.json") as config_file:
        config = json.load(config_file)

    return config

def write_to_json(minimal_set_trajectories, config):
    output_dict = {"version": VERSION, "dateGenerated": datetime.today().strftime('%Y-%m-%d'), "latticeMetadata": dict(), "primitives": []}

    for key, value in config.items():
        output_dict["latticeMetadata"][key] = value

    heading_angles = set([angle for angle in minimal_set_trajectories.keys()])
    output_dict["latticeMetadata"]["headingAngles"] = sorted(list(heading_angles))

    idx = 0
    for start_angle in minimal_set_trajectories.keys():

        for trajectory_data in minimal_set_trajectories[start_angle]:

            traj_info = dict()
            traj_info['trajectoryId'] = idx
            traj_info['startAngle'] = trajectory_data[0]
            traj_info['endAngle'] = trajectory_data[1]
            traj_info['radius'] = trajectory_data[2]
            traj_info['trajectoryLength'] = trajectory_data[3]
            traj_info['arcLength'] = trajectory_data[4]
            traj_info['straightLength'] = trajectory_data[5]
            traj_info['poses'] = trajectory_data[-1]

            output_dict["primitives"].append(traj_info)
            idx += 1

    output_dict["latticeMetadata"]["numberOfTrajectories"] = idx

    with open(config['outputFile'], 'w') as output_file:
        json.dump(output_dict, output_file)


def save_visualizations(minimal_set_trajectories):

    for start_angle in minimal_set_trajectories.keys():

        for trajectory_data in minimal_set_trajectories[start_angle]:
            path = trajectory_data[-1]
            xs, ys, _ = list(zip(*path))
            
            plt.plot(xs, ys, "b")

    plt.grid(True)
    plt.axis('square')
    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()
    plt.savefig("visualizations/all_trajectories.png")
    plt.clf()

    for start_angle in minimal_set_trajectories.keys():

        if start_angle < 0 or start_angle > 90:
            continue

        for trajectory_data in minimal_set_trajectories[start_angle]:
            path = trajectory_data[-1]
            xs, ys, _ = list(zip(*path))
            
            plt.plot(xs, ys, "b")
            plt.xlim(left_x, right_x)
            plt.ylim(left_y, right_y)
        
        plt.grid()
        plt.savefig(f'visualizations/{start_angle}.png')
        plt.clf()

if __name__ == "__main__":
    config = read_config()

    lattice_gen =  LatticeGenerator(config)
    minimal_set_trajectories = lattice_gen.run()

    write_to_json(minimal_set_trajectories, config)

    save_visualizations(minimal_set_trajectories)