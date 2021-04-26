from matplotlib.pyplot import show
import scipy.interpolate
import numpy as np
import math

import matplotlib.pyplot as plt

from helper import normalize_angle

class State:
    
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
 
    def to_numpy(self):
        return np.array([self.x, self.y, self.yaw], dtype=float)

    def arc_distance_estimate(self):
        return math.hypot(self.x, self.y)

    def __repr__(self):
        return f'State{self.x, self.y, self.yaw}'

    def copy(self):
        return State(self.x, self.y, self.yaw)

    def __hash__(self):
        return hash(f'{self.x} {self.y} {self.yaw}')

    def __eq__(self, rhs): 
        return self.x == rhs.x and self.y == rhs.y and self.yaw == rhs.yaw

class MotionModel:

    # TODO: Implement Adaptive sampling steps
    def __init__(self, config):

        self.max_steering_angle = config["MotionModel"]["max_steering_angle"]
        self.number_of_steps = config["MotionModel"]["number_of_steps"]
        self.sampling_steps = config["MotionModel"]["sampling_steps"]
    
    def update(self, state, delta_heading, step_distance):
        
        # Ensure change in heading does not exceed turn radius
        if abs(delta_heading) > np.deg2rad(self.max_steering_angle):
            sign = np.sign(delta_heading)
            # Scale the angle by step distance
            delta_heading = sign * (np.deg2rad(self.max_steering_angle) * step_distance)
            
        state.x += math.cos(state.yaw) * step_distance  
        state.y += math.sin(state.yaw) * step_distance

        state.yaw += delta_heading
        state.yaw = normalize_angle(state.yaw)
    
    def predict_motion(self, initial_state, total_distance, knot_1, knot_2):

        total_distance = abs(total_distance)

        step_distance = total_distance / self.number_of_steps

        distance_points = [0, total_distance / 3, 2* total_distance /3 , total_distance]
        curvature_points = [0, knot_1, knot_2, 0]

        # Use a spline angular velocity profile
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(distance_points, curvature_points, k=3)

        # Get number_of_steps samples in interval [0,total_distance] each equally spaced
        sample_intervals = np.linspace(0, total_distance, self.number_of_steps, endpoint=True)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        state = initial_state.copy()
        xs = [state.x]
        ys = [state.y]
        yaws = [state.yaw]

        for delta_heading in heading_inputs:
            self.update(state, delta_heading, step_distance)

            xs.append(state.x)
            ys.append(state.y)
            yaws.append(state.yaw)

        return xs, ys, yaws

    def predict_final_state(self, initial_state, total_distance, knot_1, knot_2):
        
        total_distance = abs(total_distance)

        step_distance = total_distance / self.number_of_steps

        distance_points = [0, total_distance / 3, 2 * total_distance / 3, total_distance]
        curvature_points = [0, knot_1, knot_2, 0]

        # Use a spline angular velocity profile
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(distance_points, curvature_points, k=3)

        # Get samples in interval [0,total_distance] each being step_distance apart
        sample_intervals = np.linspace(0, total_distance, self.number_of_steps, endpoint=True)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        state = initial_state.copy()

        for delta_heading in heading_inputs:
            self.update(state, delta_heading, step_distance)

        return state