from motion_model import MotionModel
from lattice_generator import LatticeGenerator
from lookup_table import LookupTable

import matplotlib.pyplot as plt

import json
import os
import csv

if __name__ == "__main__":

    with open("config.json") as config_file:
        config = json.load(config_file)

    motion_model = MotionModel(config)
    parameter_lookup_table = LookupTable(config)

    if os.path.isfile(config['LookupTable']["file_name"]):
        parameter_lookup_table.load(config['LookupTable']["file_name"])
    else:
        print("Creating Lookup Table")
        parameter_lookup_table.create()
        parameter_lookup_table.save(config['LookupTable']["file_name"])

    lattice_generator = LatticeGenerator(config)
    minimal_spanning_set = lattice_generator.generate_minimal_spanning_set(parameter_lookup_table)

    file_output = []

    fig = plt.figure()

    count = 1

    for start, color in zip(minimal_spanning_set, ['r', 'g', 'b', 'y']):
        
        ax = fig.add_subplot(2,2,count)

        for params in minimal_spanning_set[start]:
            xs1, ys1, yaws1 = motion_model.predict_motion(start, *params)
            ax.plot(xs1, ys1, color)
            ax.axis("equal")
            ax.grid(True)

            file_output.append(list(zip(xs1, ys1, yaws1)))

            # params[1] = -params[1]
            # params[2] = -params[2]

            # xs2, ys2, yaws2 = motion_model.predict_motion(State(0,0,-start.yaw), *params)
            # plt.plot(xs2, ys2, 'b')

            # file_output.append(list(zip(xs2, ys2, yaws2)))
        count += 1

    plt.show()

    # with open(config["output_file"], 'w', newline='') as lookup_table_file:
    #     writer = csv.writer(lookup_table_file, quoting=csv.QUOTE_MINIMAL)
    #     writer.writerows(file_output)



