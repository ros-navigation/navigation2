# Copyright (c) 2021, Matthew Booker
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. Reserved.

import unittest

import numpy as np
from trajectory_generator import TrajectoryGenerator

TURNING_RADIUS = 1
STEP_DISTANCE = 0.1


class TestTrajectoryGenerator(unittest.TestCase):
    """Contains the unit tests for the TrajectoryGenerator."""

    def setUp(self) -> None:
        config = {'turning_radius': TURNING_RADIUS}
        self.trajectory_generator = TrajectoryGenerator(config)

    def test_generate_trajectory_only_arc(self):
        # Quadrant 1
        end_point = np.array([1, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 2
        end_point = np.array([-1, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 3
        end_point = np.array([-1, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 4
        end_point = np.array([1, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

    def test_generate_trajectory_only_line(self):
        # Quadrant 1
        end_point = np.array([1, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(45), np.deg2rad(45), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 2
        end_point = np.array([-1, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(135), np.deg2rad(135), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 3
        end_point = np.array([-1, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(135), -np.deg2rad(135), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 4
        end_point = np.array([1, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(45), -np.deg2rad(45), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

    def test_generate_trajectory_line_to_arc(self):
        # Quadrant 1
        end_point = np.array([2, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 2
        end_point = np.array([-2, 1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 3
        end_point = np.array([-2, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 1
        end_point = np.array([2, -1])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

    def test_generate_trajectory_line_to_end(self):
        # Quadrant 1
        end_point = np.array([1, 2])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 2
        end_point = np.array([-1, 2])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 3
        end_point = np.array([-1, -2])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 4
        end_point = np.array([1, -2])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

    def test_generate_trajectory_radius_too_small(self):
        # Quadrant 1
        end_point = np.array([0.9, 0.9])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)

        # Quadrant 2
        end_point = np.array([-0.9, -0.9])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)

        # Quadrant 3
        end_point = np.array([-0.9, -0.9])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)

        # Quadrant 4
        end_point = np.array([0.9, -0.9])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), -np.deg2rad(90), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)

    def test_generate_trajectory_parallel_lines_coincident(self):
        # Quadrant 1
        end_point = np.array([5, 0])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(0), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

        # Quadrant 2
        end_point = np.array([-5, 0])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(180), STEP_DISTANCE
        )

        self.assertEqual(len(trajectory.path.xs), len(trajectory.path.ys))
        self.assertGreater(len(trajectory.path.xs), 0)

    def test_generate_trajectory_parallel_lines_not_coincident(self):
        # Quadrant 1
        end_point = np.array([0, 3])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, np.deg2rad(0), np.deg2rad(0), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)

        # Quadrant 2
        end_point = np.array([0, 3])
        trajectory = self.trajectory_generator.generate_trajectory(
            end_point, -np.deg2rad(180), -np.deg2rad(180), STEP_DISTANCE
        )

        self.assertEqual(trajectory, None)


if __name__ == '__main__':
    unittest.main()
