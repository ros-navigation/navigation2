import numpy as np
from motion_model import MotionModel
import matplotlib.pyplot as plt
import math

class TrajectoryGenerator:

    
    def __init__(self, max_iterations = 50, distance_threshold = 0.1, sampling_step = [0.5, 0.02, 0.02]):
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold # metres
        self.sampling_step = sampling_step # Used when computing the jacobian
        self.motion_model = MotionModel(.2)

    def optimize_trajectory(self, target, control_params):
        
        for i in range(self.max_iterations):

            final_predicted_state = self.motion_model.predict_final_state(*control_params)
            state_difference = target.to_numpy() - final_predicted_state.to_numpy()

            # Compute the L2 norm of the difference
            distance_to_target = np.linalg.norm(state_difference)

            if distance_to_target <= self.distance_threshold:
                print(f'Trajectory found in {i} iterations. Distance to goal is {distance_to_target}')
                pred_x, pred_y, pred_yaw = self.motion_model.predict_motion(*control_params)
                return control_params, pred_x, pred_y

            jacobian = self.compute_jacobian(target, control_params)

            try:
                control_change = - np.linalg.inv(jacobian) @ state_difference
            except np.linalg.linalg.LinAlgError:
                print("Cannot find trajectory")
                return None

            # pred_x, pred_y, pred_yaw = self.motion_model.predict_motion(*control_params)
            # show_trajectory(target, pred_x, pred_y)

            control_params += control_change

        
        print(f"Did not find suitable trajectory after {self.max_iterations} iterations")
        return None

    def compute_jacobian(self, target, control_params):

        derivatives = []

        for idx, param in enumerate(control_params):
            # Create copies to prevent altering original
            forward_params = control_params.copy()
            backward_params = control_params.copy()

            # Sample ahead/behind by amount specified in sampling step
            forward_params[idx] += self.sampling_step[idx]
            backward_params[idx] -= self.sampling_step[idx]

            # Predict motion based on changes to parameters
            forward_state = self.motion_model.predict_final_state(*forward_params)
            backward_state = self.motion_model.predict_final_state(*backward_params)

            # Calculate difference to target
            forward_diff = target.to_numpy() - forward_state.to_numpy()
            backward_diff = target.to_numpy() - backward_state.to_numpy()

            # Use central difference to numerically find derivative
            derivative_wrt_param = ((forward_diff - backward_diff) / (2 * self.sampling_step[idx])).reshape(3,1)

            derivatives.append(derivative_wrt_param)

        jacobian = np.hstack(derivatives)

        return jacobian


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def show_trajectory(target, xc, yc):  # pragma: no cover
    plt.clf()
    plot_arrow(target.x, target.y, target.yaw)
    plt.plot(xc, yc, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)