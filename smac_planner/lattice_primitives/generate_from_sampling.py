# Uses state space sampling as described in https://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2008_1/howard_thomas_2008_1.pdf

import math
from state import State
import numpy as np
from trajectory_generator import TrajectoryGenerator
import pandas as pd
import matplotlib.pyplot as plt

number_of_positions = 15
number_of_headings = 3
distance = 5
min_yaw = -45
max_yaw = 45
min_angular_range = -45
max_angular_range = 45

def create_all_states():
    states = []

    for i in range(number_of_positions):
        for j in range(number_of_headings):
            n = i * number_of_headings + j
            alpha = np.deg2rad(min_angular_range + (max_angular_range - min_angular_range) * i / (number_of_positions - 1))
            x = distance * math.cos(alpha)
            y = distance * math.sin(alpha)
            yaw = min_yaw + (max_yaw - min_yaw) * j / (number_of_headings - 1) + alpha

            states.append(State(x, y, np.deg2rad(yaw)))

    return states

def save_motion_primitives(valid_states):
    pass

def generate_motion_primitives():
    
    all_states = create_all_states()

    valid_states = []

    trajectory_gen = TrajectoryGenerator()

    for state in all_states:
        d = np.linalg.norm(state.to_numpy())

        init_p = np.array([d, 0.0, 0.0])

        result = trajectory_gen.optimize_trajectory(state, init_p)
        
        if result is not None:
            p, x_path, y_path = result
            valid_states.append(state)
            plt.plot(x_path, y_path)

    x = [state.x for state in valid_states]
    y = [state.y for state in valid_states]
    yaw = [state.yaw for state in valid_states]

    
    plt.plot(x,y, 'bo')

    # for a,b,phi in zip(x,y,yaw):
    #     plt.arrow(a,b,math.cos(phi), math.sin(phi),fc='r', ec='k', head_width=0.1, head_length=0.1)

    plt.axis('equal')
    plt.show()

    save_motion_primitives(valid_states)

if __name__ == "__main__" :
    generate_motion_primitives()