import numpy as np
from motion_model import MotionModel

class TrajectoryGenerator:

    def __init__(self, config):
        self.max_iterations = config["TrajectoryGenerator"]["max_iterations"]
        self.distance_threshold = config["TrajectoryGenerator"]["distance_threshold"] # metres
        self.rotation_threshold = config["TrajectoryGenerator"]["rotation_threshold"]
        self.verbose = config["TrajectoryGenerator"]["verbose_output"]
        self.motion_model = MotionModel(config)

    def optimize_trajectory(self, initial_state, target, control_params):
        
        for i in range(self.max_iterations):
            final_predicted_state = self.motion_model.predict_final_state(initial_state, *control_params)
            state_difference = target.to_numpy() - final_predicted_state.to_numpy()

            distance_to_target = np.linalg.norm(state_difference[:2])
            rotation_to_target = np.linalg.norm(state_difference[2])

            if distance_to_target <= self.distance_threshold and rotation_to_target <= self.rotation_threshold:
                if self.verbose:
                    print(f'Trajectory found in {i} iterations. Distance to goal is {distance_to_target}')

                pred_x, pred_y, pred_yaw = self.motion_model.predict_motion(initial_state, *control_params)

                return control_params, pred_x, pred_y, pred_yaw

            jacobian = self.compute_jacobian(initial_state, target, control_params)

            try:
                control_change = - np.linalg.inv(jacobian) @ state_difference
            except np.linalg.linalg.LinAlgError:
                if self.verbose:
                    print("Cannot find trajectory")

                return None

            control_params += control_change

        if self.verbose:
            print(f"Did not find suitable trajectory after {self.max_iterations} iterations")

        return None

    def compute_jacobian(self, initial_state, target, control_params):

        derivatives = []

        for idx in range(len(control_params)):
            # Create copies to prevent altering original
            forward_params = control_params.copy()
            backward_params = control_params.copy()

            # Sample ahead/behind by amount specified in sampling step
            forward_params[idx] += self.motion_model.sampling_steps[idx]
            backward_params[idx] -= self.motion_model.sampling_steps[idx]

            # Predict motion based on changes to parameters
            forward_state = self.motion_model.predict_final_state(initial_state, *forward_params)
            backward_state = self.motion_model.predict_final_state(initial_state, *backward_params)

            # Calculate difference to target
            forward_diff = target.to_numpy() - forward_state.to_numpy()
            backward_diff = target.to_numpy() - backward_state.to_numpy()

            # Use central difference to numerically find derivative
            derivative_wrt_param = ((forward_diff - backward_diff) / (2 * self.motion_model.sampling_steps[idx])).reshape(len(control_params),1)

            derivatives.append(derivative_wrt_param)

        jacobian = np.hstack(derivatives)

        return jacobian