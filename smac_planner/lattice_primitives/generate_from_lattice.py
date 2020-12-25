import numpy as np

minimum_turning_radius = .5 # metres
grid_separation = 0.1 # metres  (labelled delta_i)
max_curvature = 1 / minimum_turning_radius

discrete_headings = np.arange(0,360, 360/16)

def get8connected(level):

    positions = []

    level_offset = level * 2 + 1

    for i in range(-1, 2):
        for j in range(-1, 2):

            # Skip origin
            if (i == 0 and j == 0):
                continue

            x = (level_offset * i) * grid_separation
            y = (level_offset * j) * grid_separation

            positions.append((x, y))

    test = []

    for i in range(-2, 3):
        for j in range(-2, 3):
            if (i == 0 or j == 0):
                continue

            x = i
            y = j

    return positions

    (-2, -1) (-2, 1), (-1, -2) (-1, 2) (1, -2) (1, 2)

print(get8connected(3))