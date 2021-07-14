from trajectory import Trajectory, TrajectoryParameters, TrajectoryPath
import numpy as np

class TrajectoryGenerator:

        def __init__(self, config):
                self.turning_radius = config["turningRadius"]
                self.step_distance = config["stepDistance"]

        def create_arc_path(self, trajectory_params: TrajectoryParameters, step_distance: float) -> TrajectoryPath:
                '''
                Create arc trajectory using the following parameterization:
                        r(t) = <R cos(t - pi/2) + a, R sin(t - pi/2) + b>

                        R = radius
                        a = x offset
                        b = y offset

                This parameterization has the property that start_t represents the slope angle of the start point of the curve (represented in radians) and 
                end_t represents the slope angle of the end point of the curve.

                Parameters
                ----------
                trajectory_params: TrajectoryParameters
                        The parameters that describe the arc to create
                step_distance: float
                        The distance between sampled points along the arc

                Returns
                -------
                Trajectory
                        A trajectory built only from the arc portion of the trajectory parameters
                '''

                arc_length = 2 * np.pi * trajectory_params.radius * abs(trajectory_params.start_angle - trajectory_params.end_angle) / 360
                steps = int(round(arc_length / step_distance))
                
                if trajectory_params.left_turn:
                        ts = np.linspace(np.deg2rad(trajectory_params.start_angle) - np.pi/2, np.deg2rad(trajectory_params.end_angle) - np.pi/2, steps)
                else:
                        ts = np.linspace(np.deg2rad(trajectory_params.start_angle) + np.pi/2, np.deg2rad(trajectory_params.end_angle) + np.pi/2, steps)
                
                xs = trajectory_params.radius * np.cos(ts) + trajectory_params.x_offset
                ys = trajectory_params.radius * np.sin(ts) + trajectory_params.y_offset

                return TrajectoryPath(xs, ys)

        def create_line_path(self, x1, y1, x2, y2, step_distance) -> TrajectoryPath:
                '''
                Creates a 
                '''

                distance = np.linalg.norm(np.subtract([x1, y1], [x2, y2]))

                steps = int(round(distance / step_distance))

                # If steps is 0 or 1 then simply return the end point
                if steps <= 1:
                        return [x2], [y2]

                ts = np.linspace(0, 1, steps)

                xs = (1-ts) * x1 + ts * x2
                ys = (1-ts) * y1 + ts * y2

                return TrajectoryPath(xs, ys)

        def create_path(self, trajectory_params: TrajectoryParameters, step_distance: float) -> TrajectoryPath:
                if trajectory_params.radius > 0:
                        arc_trajectory = self.create_arc_path(trajectory_params, step_distance)
                
                        if trajectory_params.start_to_arc_distance > 0:
                                start_to_arc_trajectory = self.create_line_path(0,0, arc_trajectory.xs[0], arc_trajectory.ys[0], step_distance)
                        
                        if trajectory_params.arc_to_end_distance > 0:
                                arc_to_end_trajectory = self.create_line_path(arc_trajectory.xs[-1], arc_trajectory.ys[-1], *trajectory_params.end_point, step_distance)
                        
                        final_trajectory = start_to_arc_trajectory + arc_trajectory + arc_to_end_trajectory

                        return final_trajectory
                else:
                        # No arc in path therefore its only a straight line
                        line_trajectory = self.create_line_path(0,0,*trajectory_params.end_point, step_distance)
                        return line_trajectory


        def get_intersection_point(self, m1: float, c1: float, m2: float, c2: float) -> np.array:
                line1 = lambda x : m1 * x + c1

                x_point = (c2 - c1) / (m1 - m2)

                return np.array([x_point, line1(x_point)])
        
        def is_left_turn(self, m1, end_point) -> bool:
                line_eq = lambda x: m1 * x 

                if line_eq(end_point[0]) <= end_point[1]:
                        return True
                else:
                        return False

        def calculate_trajectory_params(self, end_point, start_angle, end_angle):
                '''
                Uses the following theorem to calculate the circle radius, x offset and y offset for a circle passing through (0,0) with a heading of start_angle
                and subsequently passes through point p with heading of end_angle.

                        If two segments from the same exterior point are tangent to a circle then they are congruent

                '''
                start_to_arc_distance = 0
                arc_to_end_distance = 0

                x2, y2 = end_point
                p = end_point
                q = np.array([0,0])

                # Find gradient of line 1 passing through (0,0) that makes an angle of start_angle with x_axis
                m1 = np.tan(np.deg2rad(start_angle)).round(4)

                # Find gradient of line 2 passing through (x2,y2) that makes an angle of end_angle with x-axis
                m2 = np.tan(np.deg2rad(end_angle)).round(4)

                # Deal with lines that are parallel
                if m1 == m2:
                        # If they are coincident (i.e. y-intercept is same) then simply return a circle with infinite radius
                        if round(-m2 * x2 + y2, 4) == 0:
                                return TrajectoryParameters(0, 0, 0, end_point, start_angle, end_angle, True, np.linalg.norm(end_point), 0)

                        # Deal with edge case of 90
                        elif abs(start_angle) == 90 and p[0] == q[0]:
                                return TrajectoryParameters(0, 0, 0, end_point, start_angle, end_angle, True, np.linalg.norm(end_point), 0)
                        else:
                                print(f'No trajectory possible for equivalent start and end angles that also passes through p = {x2, y2}')
                                return None

                angle_between_lines = np.deg2rad(180 - abs(end_angle - start_angle))
                min_valid_distance = round(self.turning_radius / np.tan(angle_between_lines / 2), 4)

                # Find intersection point of the lines 1 and 2
                intersection_point = self.get_intersection_point(m1, 0, m2, -m2 * x2 + y2)

                if intersection_point[0] < 0:
                        print("No trajectory possible as intersection point occurs behind start point")
                        return None

                m2_direction_vec = np.array([1, m2])

                if end_angle > 90 or end_angle < -90:
                        m2_direction_vec *= -1

                if np.all(m2_direction_vec != np.zeros(2)):
                        m2_direction_vec /= np.linalg.norm(m2_direction_vec) 

                p_direction_vec = p - intersection_point

                if np.all(p_direction_vec != np.zeros(2)):
                        p_direction_vec /= np.linalg.norm(p_direction_vec)

                if np.dot(m2_direction_vec, p_direction_vec) < 0:
                        print("No trajectory possible since end point occurs before intersection along line 2")
                        return None

                # Calculate distance between point p = (x2,y2) and intersection point
                dist_p = round(np.linalg.norm(p-intersection_point, axis=0),4)

                # Calculate distance between point q = (0,0) and intersection point
                dist_q = round(np.linalg.norm(q-intersection_point),4)

                # Both the distance of p along line 2 and intersection point along line 1 must be greater than the minimum valid distance
                if dist_p < min_valid_distance or dist_q < min_valid_distance:
                        print("No trajectory possible where radius is larger than minimum turning radius")
                        return None
                
                if dist_q < dist_p:
                        # Find new point p on line 2 that is equidistant away from intersection point as point q is on line 1
                        vec_line2 = p - intersection_point
                        vec_line2 /= np.linalg.norm(vec_line2)
                        p = intersection_point + dist_q * vec_line2

                        arc_to_end_distance = dist_p - dist_q

                elif dist_q > dist_p:
                        # Find new point q on line 1 that is equidistant away from intersection point as point p is on line 2
                        vec_line1 = -intersection_point
                        vec_line1 /= np.linalg.norm(vec_line1)

                        q = intersection_point + dist_p * vec_line1

                        start_to_arc_distance = dist_q - dist_p

                x1, y1 = q
                x2, y2 = p

                # Find intersection point of the perpindicular lines of line 1 and 2 that pass through q and p respectively
                if m1 == 0:
                        # If line 1 has gradient 0 then it is the x-axis.

                        perp_line2 = lambda x: -1/m2 * (x - x2) + y2 
                        circle_center = np.array([x1, perp_line2(x1)])
                elif m2 == 0:
                        perp_line1 = lambda x: -1/m1 * (x - x1) + y1
                        circle_center = np.array([x2, perp_line1(x2)])
                else:
                        perp_m1 = -1/m1 if m1 != 0 else 0
                        perp_m2 = -1/m2 if m2 != 0 else 0
                        
                        circle_center = self.get_intersection_point(perp_m1, -perp_m1 * x1 + y1, perp_m2, -perp_m2 * x2 + y2)
 
                # The circles radius is the length from the center to point p (or q)
                radius = round(np.linalg.norm(circle_center - p), 4)
                x_offset = round(circle_center[0], 4)
                y_offset = round(circle_center[1], 4)

                if radius < self.turning_radius:
                        print(f'Calculated circle radius is smaller than allowed turning radius: r = {radius}, min_radius = {self.turning_radius}')
                        return None

                left_turn = self.is_left_turn(m1, end_point)

                return TrajectoryParameters(radius, x_offset, y_offset, end_point, start_angle, end_angle, left_turn, start_to_arc_distance, arc_to_end_distance)

        def generate_trajectory(self, end_point, start_angle, end_angle, step_distance=None):

                if step_distance is None:
                        step_distance = self.step_distance

                trajectory_params = self.calculate_trajectory_params(end_point, start_angle, end_angle)

                if trajectory_params is None:
                        return [], [], trajectory_params

                trajectory_path = self.create_path(trajectory_params, step_distance)

                return Trajectory(trajectory_path, trajectory_params)