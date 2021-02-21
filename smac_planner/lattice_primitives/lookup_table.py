from motion_model import State
from trajectory_generator import TrajectoryGenerator
import matplotlib.pyplot as plt

import csv
import numpy as np
from collections import defaultdict
import math

class LookupTable:

    def __init__(self, config):
        self.trajectory_generator = TrajectoryGenerator(config)


        self.headings = [0.0, math.atan(1/2), np.deg2rad(45), math.atan(2), np.deg2rad(90)]

        self.table = defaultdict(list)

    def get_states_list(self):
        x = np.arange(0, 10, 1)
        y = np.arange(0, 10, 1)

        yaw = [-np.deg2rad(30), 0, np.deg2rad(30)]

        states = []

        for x_pos in x:
            for y_pos in y:

                # Skip the origin point
                if x_pos == 0 and y_pos == 0:
                    continue

                for heading in yaw:
                    states.append(State(x_pos, y_pos, heading))

        return states

    def get_nearest_params(self, start_state, end_state):
        min_distance = float('inf')

        best_params = self.table[start_state][0][1:]

        for values in self.table[start_state]:
            state_from_table = values[0]
            diff = np.linalg.norm(state_from_table.to_numpy() - end_state.to_numpy())

            if diff < min_distance:
                min_distance = diff
                best_params = values[1:]

        return best_params

    def create(self):
        # Create an initial set of parameters (1 for each heading)
        self.table[State(0,0,self.headings[0])].append(((State(1,0,self.headings[0]), 1,0,0,1)))
        self.table[State(0,0,self.headings[1])].append(((State(2,1,self.headings[1]), math.sqrt(5),0,0,1)))
        self.table[State(0,0,self.headings[2])].append(((State(1,1,self.headings[2]), math.sqrt(2),0,0,1)))
        self.table[State(0,0,self.headings[3])].append(((State(1,2,self.headings[3]), math.sqrt(5),0,0,1)))
        self.table[State(0,0,self.headings[4])].append(((State(0,1,self.headings[4]), 1,0,0,1)))

        states_list = self.get_states_list()
        
        test = [] # TODO: REMOVE

        failed_states = 0 

        initial_states = [State(0,0,i) for i in self.headings]

        for start in initial_states:
            for target in states_list:
                best_params = self.get_nearest_params(start, target)
                estimated_p = [target.arc_distance_estimate(), best_params[1], best_params[2], best_params[3]]
                
                result = self.trajectory_generator.optimize_trajectory(start, target, estimated_p)

                if result is not None:
                    p, xs, ys, yaws = result 
                    self.table[start].append((target, *p))
                    test.append((xs, ys)) # TODO: REMOVE
                else:
                    failed_states += 1

        # TODO: REMOVE
        for x_path, y_path in test:
            plt.plot(x_path, y_path, "-r")
        
        plt.show()

        print(f'{(1 - failed_states / (len(states_list) * len(initial_states))) * 100} % states succesfully generated')

    def save(self, file_name):
        # Save the values by unpacking state objects
        formatted_table = []

        for start_state, values in self.table.items():
            for motion in values:
                formatted_table.append([start_state.x, start_state.y, start_state.yaw, motion[0].x, motion[0].y, motion[0].yaw, motion[1], motion[2], motion[3], motion[4]])

        with open(file_name, 'w', newline='') as lookup_table_file:
            writer = csv.writer(lookup_table_file, quoting=csv.QUOTE_MINIMAL)
            writer.writerows(formatted_table)

    def load(self, file_path):
        with open(file_path, newline='', encoding='utf-8') as lookup_table_file:
            reader = csv.reader(lookup_table_file, quoting=csv.QUOTE_MINIMAL)

            for row in reader:
                start_x, start_y, start_yaw, target_x, target_y, target_yaw, arc_distance, k0, k1, time = row
                self.table[State(int(start_x), int(start_y), float(start_yaw))].append([State(int(target_x), int(target_y), float(target_yaw)), float(arc_distance), float(k0), float(k1), float(time)])