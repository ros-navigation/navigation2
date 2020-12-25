import scipy.interpolate
import numpy as np
import math
from state import State

import matplotlib.pyplot as plt

class MotionModel:

    def __init__(self, wheel_base=1, traversal_velocity = 1.0, step_distance = 0.1):

        self.wheel_base = wheel_base # metres
        self.traversal_velocity = traversal_velocity # metres / second
        self.step_distance = step_distance # metres

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def update(self, state, delta_heading, delta_time):
        # Uses a bicycle model with center at rear
        state.velocity = self.traversal_velocity
        state.x += state.velocity * math.cos(state.yaw) * delta_time
        state.y += state.velocity * math.sin(state.yaw) * delta_time
        state.yaw += state.velocity / self.wheel_base * math.tan(delta_heading) * delta_time
        state.yaw = MotionModel._normalize_angle(state.yaw)
    
    def predict_motion(self, s, km, kf):

        s = s if s > 0 else 1

        number_of_steps = s / self.step_distance

        total_time = s / self.traversal_velocity

        time_points = [0, total_time / 2, total_time]
        curvature_points = [0, km, kf]

        # Use a spline angular velocity profile
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(time_points, curvature_points, k=2)

        # Generate sampling points
        sample_intervals = np.arange(0, total_time, total_time/number_of_steps)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        delta_time = total_time / number_of_steps

        state = State(0,0,0)
        xs = [state.x]
        ys = [state.y]
        yaws = [state.yaw]

        for delta_heading in heading_inputs:
            self.update(state, delta_heading, delta_time)

            xs.append(state.x)
            ys.append(state.y)
            yaws.append(state.yaw)

        return xs, ys, yaws

    def predict_final_state(self, s, km, kf):
        
        s = s if s > 0 else 1

        number_of_steps = s / self.step_distance

        total_time = s / self.traversal_velocity

        time_points = [0, total_time / 2, total_time]
        curvature_points = [0, km, kf]

        # Use a spline angular velocity profile since scale of inputs is same
        angular_velocity_profile = scipy.interpolate.UnivariateSpline(time_points, curvature_points, k=2)

        # Generate sampling points
        sample_intervals = np.arange(0, total_time, total_time/number_of_steps)

        # Find heading at specified sampling points
        heading_inputs = [angular_velocity_profile(i) for i in sample_intervals]

        delta_time = total_time / number_of_steps

        state = State(0,0,0)

        for delta_heading in heading_inputs:
            self.update(state, delta_heading, delta_time)

        return state