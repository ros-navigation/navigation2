import numpy as np


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angle_difference(angle_1, angle_2, is_left_turn=None):

    if is_left_turn is None:
        dif = abs(angle_1 - angle_2)

        return dif if dif <= np.pi else 2 * np.pi - dif

    elif is_left_turn:

        if angle_2 >= angle_1:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)

    else:
        if angle_1 >= angle_2:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)
