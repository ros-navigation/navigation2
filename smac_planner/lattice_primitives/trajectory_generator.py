from trajectory_params import TrajectoryParameters
import numpy as np
import matplotlib.pyplot as plt

import math

class TrajectoryGenerator:

        def __init__(self, config):
                self.turning_radius = config["TrajectoryGenerator"]["turning_radius"]
                self.number_of_steps = config["TrajectoryGenerator"]["number_of_steps"]


        def create_arc_trajectory(self, radius, x_offset, y_offset, start_t, end_t, steps):

                '''
                Create arc trajectory using the following parameterization:
                        r(t) = <R cos(t - pi/2) + a, R sin(t - pi/2) + b>

                        R = radius
                        a = x offset
                        b = y offset

                This parameterization has the property that start_t represents the slope angle of the start point of the curve (represented in radians) and 
                end_t represents the slope angle of the end point of the curve.
                '''

                ts = np.linspace(np.deg2rad(start_t) - np.pi/2, np.deg2rad(end_t) - np.pi/2, steps)
                
                xs = radius * np.cos(ts) + x_offset
                ys = radius * np.sin(ts) + y_offset

                return xs, ys

        def create_line_trajectory(self, x1, y1, x2, y2, steps):
                ts = np.linspace(0, 1, steps)

                xs = (1-ts) * x1 + ts * x2
                ys = (1-ts) * y1 + ts * y2

                return xs, ys

        def build_trajectory(self, trajectory_params):

                arc_length = 2 * np.pi * trajectory_params.radius * abs(trajectory_params.start_angle - trajectory_params.end_angle) / 360
                total_distance = trajectory_params.start_to_arc_distance + trajectory_params.arc_to_end_distance + arc_length

                if trajectory_params.radius > 0:
                        steps_for_arc = math.ceil(self.number_of_steps * arc_length / total_distance)
                        steps_for_start = math.floor(self.number_of_steps * trajectory_params.start_to_arc_distance / total_distance)
                        steps_for_end = math.floor(self.number_of_steps * trajectory_params.arc_to_end_distance / total_distance)

                        arc_xs, arc_ys = self.create_arc_trajectory(trajectory_params.radius, trajectory_params.x_offset, trajectory_params.y_offset, \
                                                                trajectory_params.start_angle, trajectory_params.end_angle, steps_for_arc)

                        start_xs, start_ys = self.create_line_trajectory(0,0, arc_xs[0], arc_ys[0], steps_for_start)
                        end_xs, end_ys = self.create_line_trajectory(arc_xs[-1], arc_ys[-1], *trajectory_params.end_point, steps_for_end)
                        
                        return np.concatenate((start_xs, arc_xs, end_xs)), np.concatenate((start_ys, arc_ys, end_ys))
                else:
                        xs, ys = self.create_line_trajectory(0,0,*trajectory_params.end_point, self.number_of_steps)
                        return xs, ys


        def get_intersection_point(self, m1, c1, m2, c2):
                line1 = lambda x : m1 * x + c1

                x_point = (c2 - c1) / (m1 - m2)

                return np.array([x_point, line1(x_point)])

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
                m1 = round(np.tan(np.deg2rad(start_angle)), 4)

                # Find gradient of line 2 passing through (x2,y2) that makes an angle of end_angle with x-axis
                m2 = round(np.tan(np.deg2rad(end_angle)), 4)

                # Deal with lines that are parallel
                if m1 == m2:
                        # If they are coincident then simply return a circle with infinite radius
                        if round(-m2 * x2 + y2, 4) == 0:
                                return TrajectoryParameters(-1, -1, -1, end_point, start_angle, end_angle, 0, 0)
                        else:
                                print(f'No trajectory possible for equivalent start and end angles that also passes through p = {x2, y2}')
                                return None


                angle_between_lines = np.deg2rad(180 - (end_angle - start_angle))
                min_valid_distance = round(self.turning_radius / np.tan(angle_between_lines / 2), 4)

                # Find intersection point of the lines 1 and 2
                intersection_point = self.get_intersection_point(m1, 0, m2, -m2 * x2 + y2)

                if intersection_point[0] < 0:
                        print("No trajectory possible as intersection point occurs behind start point")
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
                        # Find new point p on line 2 that is equidistant away from intersection point as point p is on line 2
                        vec_line2 = np.array([1, m2]) /  np.linalg.norm(np.array([1, m2]))
                        p = intersection_point + dist_q * vec_line2

                        arc_to_end_distance = dist_p - dist_q

                elif dist_q > dist_p:
                        # Find new point q on line 1 that is equidistant away from intersection point as point p is on line 2
                        vec_line1 = intersection_point / np.linalg.norm(intersection_point)
                        q = intersection_point - dist_p * vec_line1

                        start_to_arc_distance = dist_q - dist_p

                x1, y1 = q
                x2, y2 = p

                # Find intersection point of the perpindicular lines of line 1 and 2 that pass through q and p respectively
                if m1 == 0:
                        # If line 1 has gradient 0 then it is the x-axis.

                        perp_line2 = lambda x: -1/m2 * (x - x2) + y2 
                        circle_center = np.array([x1, perp_line2(x1)])
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

                return TrajectoryParameters(radius, x_offset, y_offset, end_point, start_angle, end_angle, start_to_arc_distance, arc_to_end_distance)

        def generate_trajectory(self, end_point, start_angle, end_angle):
                trajectory_params = self.calculate_trajectory_params(end_point, start_angle, end_angle)

                if trajectory_params is None:
                        return [], []

                return self.build_trajectory(trajectory_params)

if __name__ == '__main__':
        p = np.array([5,5])
        print(p)
        start_angle = 45
        end_angle = 45
        trajGen = TrajectoryGenerator({"TrajectoryGenerator":{"turning_radius":1,"number_of_steps":100}})
        xs, ys = trajGen.generate_trajectory(p, start_angle, end_angle)

        plt.plot(xs, ys)
        plt.grid()
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.show()
