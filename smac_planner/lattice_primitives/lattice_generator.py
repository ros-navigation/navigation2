from typing import List, Dict, Tuple
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

from helper import angle_difference
from lookup_table import LookupTable
from motion_model import State
from trajectory_generator import TrajectoryGenerator

import math
import time

class LatticeGenerator:

    def __init__(self, config) -> None:
        self.grid_separation = config["LatticeGenerator"]["grid_separation"]

        self.trajectory_generator = TrajectoryGenerator(config)

    def is_minimal_path(self, xs: List[float], ys: List[float], yaws: List[float], minimal_spanning_trajectories: Dict[float, State]) -> bool:

        path_pose = sorted(list(zip(xs,ys,yaws)), key=lambda c: (c[0], c[1]))
        distance_threshold = 0.5 * self.grid_separation
        rotation_threshold = 0.5 * np.deg2rad(360 / 16)

        idx = 0

        for x_line, bin in minimal_spanning_trajectories.items():

            # Skip any positions in the path that are not in range of this bin
            while path_pose[idx][0] < x_line - distance_threshold:
                idx += 1

                # If we reached the end of the path then the path must be in the minimal set
                if idx >= len(path_pose):
                    return True

            while path_pose[idx][0] <= x_line + distance_threshold:

                x, y, yaw = path_pose[idx]

                for visited_state in bin:

                    # If we are within the distance + rotation thresholds then this path can be composed of the path that reaches the current state -> not a minimal path
                    if math.hypot(x - visited_state.x, y - visited_state.y) < distance_threshold and angle_difference(yaw, visited_state.yaw) < rotation_threshold:
                        return False
                
                # Move to next position in path
                idx += 1

                # If we reached the end of the path then the path must be in the minimal set
                if idx >= len(path_pose):
                    return True


        return True


    def get_lattice_points_at_level(self, level: int) -> List[Tuple[float, float]]:
        positions = []

        max_point_coord = self.grid_separation * level

        for i in range(0, level):
            varying_point_coord = self.grid_separation * i

            # Varying y-coord
            positions.append((max_point_coord, varying_point_coord))
            
            # Varying x-coord
            positions.append((varying_point_coord, max_point_coord))


        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return positions

    def generate_minimal_spanning_set(self, lookup_table: LookupTable):
        s = time.time()
        result = defaultdict(list)

        initial_headings = [math.atan(i) for i in [0, 1/2, 1, 2]]

        # Create initial states using (0,0) and discrete headings
        initial_states = [State(x=0, y=0, yaw=heading) for heading in initial_headings]

        # Use from -180 to 180 with 0 being straight (otherwise lookup will be funny)
        target_headings = [np.deg2rad(quadrants) + heading for quadrants in [-180, -90, 0, 90] for heading in initial_headings]

        for start in initial_states:
            
            minimal_spanning_trajectories = defaultdict(list) # TODO: Rename - not clear
            none_found = True
            all_decomposed = False
            current_level = 1

            # TODO: Minimum number of initial iterations before exiting? (with the none_found flag this loop may never exit)
            while ((not all_decomposed) or none_found) and current_level < 4:
                all_decomposed = True

                # Generate x,y coordinates for current level
                positions = self.get_lattice_points_at_level(current_level)

                # Create target states using x,y coordinates and discrete headings
                targets = [State(x=p[0], y=p[1], yaw=heading) for p in positions for heading in target_headings]

                for target in targets:
                    print(start, target)
                    best_params = lookup_table.get_nearest_params(start, target)
                    trajectory = self.trajectory_generator.optimize_trajectory(start, target, best_params)

                    if trajectory is not None:
                        none_found = False
                        params, xs, ys, yaws = trajectory

                        # Check if path overlaps something in minimal spanning set
                        if(self.is_minimal_path(xs, ys, yaws, minimal_spanning_trajectories)):
                            minimal_spanning_trajectories[target.x].append(target)
                            all_decomposed = False

                            result[start].append(params)

                current_level += 1

        print("Time elapsed = ", time.time() - s)    

        return result