import unittest
from trajectory_generator import TrajectoryGenerator
import numpy as np

TURNING_RADIUS = 1
NUMBER_OF_STEPS = 100

class TestTrajectoryGenerator(unittest.TestCase):

    def setUp(self) -> None:
        config = {"TrajectoryGenerator":{"turning_radius":TURNING_RADIUS,"number_of_steps":NUMBER_OF_STEPS}}
        self.trajectory_generator = TrajectoryGenerator(config)

    def test_generate_trajectory_only_arc(self):
        end_point = np.array([1,1])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 90)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), NUMBER_OF_STEPS)
    
    def test_generate_trajectory_only_line(self):
        end_point = np.array([1,1])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 45, 45)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), NUMBER_OF_STEPS)

    def test_generate_trajectory_line_to_arc(self):
        end_point = np.array([2,1])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 90)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), NUMBER_OF_STEPS)

    def test_generate_trajectory_line_to_end(self):
        end_point = np.array([1,2])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 90)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), NUMBER_OF_STEPS)

    def test_generate_trajectory_radius_too_small(self):
        end_point = np.array([.9,.9])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 90)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), 0)

    def test_generate_trajectory_parallel_lines_coincident(self):
        end_point = np.array([5, 0])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 0)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), NUMBER_OF_STEPS)

    def test_generate_trajectory_parallel_lines_not_coincident(self):
        end_point = np.array([0, 3])
        xs, ys = self.trajectory_generator.generate_trajectory(end_point, 0, 0)

        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), 0)

if __name__ == '__main__':
    unittest.main()