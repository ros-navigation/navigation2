import numpy as np


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angle_difference(angle_1, angle_2):

    difference = abs(angle_1 - angle_2)

    if difference > np.pi:
        # If difference > 180 return the shorter distance between the angles
        difference = 2*np.pi - difference

    return difference
