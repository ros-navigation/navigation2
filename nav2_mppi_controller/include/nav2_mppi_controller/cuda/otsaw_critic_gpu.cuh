// Copyright (c) 2024 OTSAW

#ifndef NAV2_MPPI_CONTROLLER__CUDA__OTSAW_CRITIC_GPU_CUH_
#define NAV2_MPPI_CONTROLLER__CUDA__OTSAW_CRITIC_GPU_CUH_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

std::vector<float> calc_constraint_critics_cost(
    std::vector<float> vx,
    std::vector<float> vy,
    float max_vel,
    float min_vel,
    float dt
);

void calc_obstacle_critics_cost(
  unsigned char * costmap_arr,
  unsigned int costmap_size_x,
  unsigned int costmap_size_y,
  double costmap_resolution,
  double costmap_origin_x,
  double costmap_origin_y,
  std::vector<float> traj_x;
  std::vector<float> traj_y;
  std::vector<float> traj_yaws;
  unsigned int batch_size,
  unsigned int time_steps,
  std::vector<float>& raw_cost;
  std::vector<float>& repulsive_cost;
);

#endif  // NAV2_MPPI_CONTROLLER__CUDA__OTSAW_CRITIC_GPU_CUH_
