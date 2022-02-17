from nav2_msgs.action import ComputePathToPose
import numpy as np
import math

import os
from ament_index_python.packages import get_package_share_directory
import pickle

import seaborn as sns
import matplotlib.pylab as plt
from tabulate import tabulate

def getPaths(results):
    paths = [] 
    for result in results:
        for path in result:
            paths.append(path.path)
    return paths

def getTimes(results):
    times = []
    for result in results:
        for time in result:
            times.append(time.planning_time.nanosec)
    return times

def getMapCoordsFromPaths(paths, resolution):
    coords = []
    for path in paths:
        x = []
        y = []
        for pose in path.poses:
            x.append(pose.pose.position.x/resolution)
            y.append(pose.pose.position.y/resolution)
        coords.append(x)
        coords.append(y)
    return coords

def getPathLength(path):
    path_length = 0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        path_length = path_length + math.sqrt((x_curr-x_prev)**2 + (y_curr-y_prev)**2)
        x_prev = x_curr
        y_prev = y_curr
    return path_length

def plotResults(costmap, paths):
    coords = getMapCoordsFromPaths(paths, costmap.metadata.resolution)
    data = np.asarray(costmap.data)
    data.resize(costmap.metadata.size_y, costmap.metadata.size_x)
    data = np.where(data < 254, 0, data)
    data = data * -1
    plt.figure(3)
    ax = sns.heatmap(data, cbar=False)
    for i in range(0, len(coords), 2):
        ax.plot(coords[i], coords[i+1], linewidth=0.7)
    plt.axis('off')
    plt.show()


def main():

    nav2_planner_metrics_dir = get_package_share_directory('nav2_planner_metrics')

    with open(os.path.join(nav2_planner_metrics_dir, 'costmap.pickle'), 'rb') as f:
        costmap = pickle.load(f)

    with open(os.path.join(nav2_planner_metrics_dir, 'results.pickle'), 'rb') as f:
        results = pickle.load(f)
    
    with open(os.path.join(nav2_planner_metrics_dir, 'planners.pickle'), 'rb') as f:
        planners = pickle.load(f) 

    paths = getPaths(results)
    path_lengths = []

    for path in paths:
        path_lengths.append( getPathLength(path))
    path_lengths = np.asarray(path_lengths)
    total_paths = len(paths)

    path_lengths.resize((int(total_paths/len(planners)), len(planners)))
    path_lengths = np.transpose(path_lengths)
    print(path_lengths.size)

    times = getTimes(results)
    times = np.asarray(times)
    times = times * 0.000001; # convert nano to milli
    times.resize((int(total_paths/len(planners)), len(planners)))
    times = np.transpose(times)

    # Generate table
    planner_table = [['Planner', 'Average Path Length (m)', 'Average Time (ms)']]
    for i in range(0,len(planners)):
        planner_table.append([planners[i], np.average(path_lengths[i]), np.average(times[i])])

    #Visualize results
    print(tabulate(planner_table))
    # plotResults(costmap, paths)

if __name__ == '__main__':
    main()