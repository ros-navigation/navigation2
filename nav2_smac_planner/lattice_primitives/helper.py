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

import numpy as np


def normalize_angle(angle):
    """
    Normalize the angle to between [0, 2pi).

    Args:
    angle: float
        The angle to normalize in radians

    Returns
    -------
    The normalized angle in the range [0,2pi)

    """
    while angle >= 2 * np.pi:
        angle -= 2 * np.pi

    while angle < 0:
        angle += 2 * np.pi

    return angle


def angle_difference(angle_1, angle_2, left_turn=None):
    """
    Calculate the difference between two angles based on a given direction.

    Args:
    angle_1: float
        The starting angle in radians
    angle_2: float
        The ending angle in radians
    left_turn: bool
        The direction of turn. True if left, false if right
        and None if smallest angular difference should be
        returned

    Returns
    -------
    The angular difference between the two angles according to
    the specified turn direction

    """
    if left_turn is None:
        dif = abs(angle_1 - angle_2)

        return dif if dif <= np.pi else 2 * np.pi - dif

    elif left_turn:

        if angle_2 >= angle_1:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)

    else:
        if angle_1 >= angle_2:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)


def interpolate_yaws(start_angle, end_angle, left_turn, steps):
    """
    Create equally spaced yaws between two angles.

    Args:
    start_angle: float
        The starting angle
    end_angle: float
        The ending angle
    left_turn: bool
        The direction of turn. True if left, False otherwise
    steps: int
        The number of yaws to generate between start and end
        angle

    Returns
    -------
    An array of yaws starting at start angle and ending at end
    angle with steps number of angles between them

    """
    if left_turn:
        if start_angle > end_angle:
            end_angle += 2 * np.pi
    else:
        if end_angle > start_angle:
            end_angle -= 2 * np.pi

    yaws = np.linspace(start_angle, end_angle, steps)
    yaws = np.vectorize(normalize_angle)(yaws)

    return yaws


def get_rotation_matrix(angle):
    """
    Return a rotation matrix that is equivalent to a 2D rotation of angle.

    Args:
    angle: float
        The angle to create a rotation matrix for

    Returns
    -------
    A 2x2 matrix representing a 2D rotation by angle

    """
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
