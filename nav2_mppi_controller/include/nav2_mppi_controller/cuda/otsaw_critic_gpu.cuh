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

#endif  // NAV2_MPPI_CONTROLLER__CUDA__OTSAW_CRITIC_GPU_CUH_
