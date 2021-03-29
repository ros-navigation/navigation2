import scipy.interpolate
import numpy as np
import math

import matplotlib.pyplot as plt

from helper import normalize_angle

class State:
    
    def __init__(self, x, y, yaw, velocity = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity

    def to_numpy(self):
        return np.array([self.x, self.y, self.yaw, self.velocity], dtype=float)

    def arc_distance_estimate(self):
        return math.hypot(self.x, self.y)

    def __repr__(self):
        return f'State{self.x, self.y, self.yaw, self.velocity}'

    def copy(self):
        return State(self.x, self.y, self.yaw, self.velocity)

    def __hash__(self):
        return hash(f'{self.x} {self.y} {self.yaw}')

    def __eq__(self, rhs): 
        return self.x == rhs.x and self.y == rhs.y and self.yaw == rhs.yaw and self.velocity == rhs.velocity

class MotionModel:

    # TODO: Implement Adaptive sampling steps
    def __init__(self, config):

        self.wheel_base = config["MotionModel"]["wheel_base"] # metres
        self.max_turn_rate = config["MotionModel"]["max_turn_rate"]
        self.step_distance = config["MotionModel"]["step_distance"] # metres
        self.sampling_steps = config["MotionModel"]["sampling_steps"]
    
    def update(self, state, velocity, delta_heading, delta_time):

        delta_yaw = (state.velocity / self.wheel_base) * math.tan(delta_heading) * delta_time

        # Ensure turn rate is not greater than max turn rate
        if (abs(delta_yaw) > np.deg2rad(self.max_turn_rate)):
            sign = 1 if delta_yaw >= 0 else -1
            delta_yaw = np.deg2rad(self.max_turn_rate) * sign

        # Uses a bicycle model with center at rear
        state.velocity = velocity

        state.x += state.velocity * math.cos(state.yaw) * delta_time
        state.y += state.velocity * math.sin(state.yaw) * delta_time
        state.yaw += delta_yaw
        state.yaw = normalize_angle(state.yaw)
    
    def predict_motion(self, initial_state, arc_length, knot_1, knot_2, total_time):

        arc_length = max(1, abs(arc_length))
        total_time = max(1, total_time)

        number_of_steps = arc_length / self.step_distance

        time_points = [0, total_time / 2, total_time]
        curvature_points = [0, knot_1, knot_2]
        linear_velocity = arc_length / total_time

        # Use a spline angular velocity profile
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(time_points, curvature_points, k=2)

        # Generate sampling points
        sample_intervals = np.arange(0, total_time, total_time/number_of_steps)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        delta_time = total_time / number_of_steps

        state = initial_state.copy()
        xs = [state.x]
        ys = [state.y]
        yaws = [state.yaw]

        for delta_heading in heading_inputs:
            self.update(state, linear_velocity, delta_heading, delta_time)

            xs.append(state.x)
            ys.append(state.y)
            yaws.append(state.yaw)

        return xs, ys, yaws

    def predict_final_state(self, initial_state, arc_length, knot_1, knot_2, total_time):

        arc_length = max(1, abs(arc_length))
        total_time = max(1, total_time)

        number_of_steps = arc_length / self.step_distance

        time_points = [0, total_time / 2, total_time]
        curvature_points = [0, knot_1, knot_2]

        linear_velocity = arc_length / total_time

        # Use a spline angular velocity profile
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(time_points, curvature_points, k=2)

        # Generate sampling points
        sample_intervals = np.arange(0, total_time, total_time/number_of_steps)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        delta_time = total_time / number_of_steps

        state = initial_state.copy()
        
        for delta_heading in heading_inputs:
            self.update(state, linear_velocity, delta_heading, delta_time)

        return state