// Copyright (c) 2024 OTSAW

#ifndef NAV2_MPPI_CONTROLLER__CUDA__OTSAW_UTILS_CUH_
#define NAV2_MPPI_CONTROLLER__CUDA__OTSAW_UTILS_CUH_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "nav2_mppi_controller/cuda/line_iterator.cuh"

void calc_cost_at_pose(
    // Input(0): Trajectories
    std::vector<float> traj_x,
    std::vector<float> traj_y,
    std::vector<float> traj_yaws,
    unsigned int batch_size,
    unsigned int time_steps,
    // Input(1): Costmap
    unsigned char * costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    float costmap_resolution,
    float costmap_origin_x,
    float costmap_origin_y,
    // Input(2): Footprint
    std::vector<float> footprint_x,
    std::vector<float> footprint_y,
    unsigned int footprint_size,
    // Input(3): Config
    bool consider_footprint,
    float possibly_inscribed_cost,
    // Output:
    std::vector<float>& pose_cost,
    std::vector<bool>& using_footprint
);

#endif  // NAV2_MPPI_CONTROLLER__CUDA__OTSAW_UTILS_CUH_
