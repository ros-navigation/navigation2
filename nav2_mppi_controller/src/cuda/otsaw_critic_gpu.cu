// Copyright (c) 2024 OTSAW

#include "nav2_mppi_controller/cuda/otsaw_critic_gpu.cuh"

__global__ void constraintKernel(
    float *out,
    float *vx,
    float *vy,
    float max_vel,
    float min_vel,
    float dt,
    int n
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < n) {
        float sgn = (vx[tid] > 0.0) ? 1.0 : -1.0;
        float vel_total = sgn * sqrtf(vx[tid]*vx[tid] + vy[tid]*vy[tid]);
        
        float total_error = 0.0;
        if (vel_total > max_vel) total_error += (vel_total - max_vel);
        if (vel_total < min_vel) total_error += (min_vel - vel_total);
        
        // TODO: support ackerman model

        // Calculate cost output
        out[tid] = (total_error) * dt;

        // printf("tid=%d, vx=%.2f, range=(%.2f, %.2f), vel_total=%.2f, total_error=%.2f, out=%.2f\n",
        //     tid, vx[tid], 
        //     min_vel, max_vel,
        //     vel_total,
        //     total_error,
        //     out[tid]);
    }
}

std::vector<float> calc_constraint_critics_cost(
    std::vector<float> vx,
    std::vector<float> vy,
    float max_vel,
    float min_vel,
    float dt
) {
    int N = vx.size();
    thrust::device_vector<float> d_vx = vx;
    thrust::device_vector<float> d_vy = vy;
    thrust::device_vector<float> d_out(N);

    // Extract raw pointersvec_vx
    float* raw_ptr_vx = thrust::raw_pointer_cast(d_vx.data());
    float* raw_ptr_vy = thrust::raw_pointer_cast(d_vy.data());
    float* raw_ptr_out = thrust::raw_pointer_cast(d_out.data());

    // Launch kernel
    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;
    constraintKernel<<<numBlocks, blockSize>>>(
        raw_ptr_out,
        raw_ptr_vx,
        raw_ptr_vy,
        max_vel,
        min_vel,
        dt,
        N
    );

    cudaDeviceSynchronize();

    thrust::host_vector<float> h_out = d_out;
    std::vector<float> out(h_out.begin(), h_out.end());

    return out;
}

__global__ void obstacleKernel(
    unsigned char *costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    float *traj_x,
    float *traj_y,
    float *traj_yaws,
    unsigned int batch_size,
    unsigned int time_steps,
    float *raw_cost,
    float *repulsive_cost,
    int n
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < n) {
        unsigned int j = tid / batch_size;  // pose in traj index
        unsigned int i = tid - (j * batch_size);  // traj in batch of trajectories

        // Process
        printf("tid=%d, i=%d, j=%d, x=%.2f, y=%.2f, yaws=%.2f\n",
            tid,
            i,
            j,
            traj_x[tid],
            traj_y[tid],
            traj_yaws[tid]
        );
    }
}

void calc_obstacle_critics_cost(
    unsigned char * costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    std::vector<float> traj_x,
    std::vector<float> traj_y,
    std::vector<float> traj_yaws,
    unsigned int batch_size,
    unsigned int time_steps,
    std::vector<float>& raw_cost,
    std::vector<float>& repulsive_cost
) {
    // Max number of triggers
    int N = traj_x.size();
    // printf("traj_x.size()=%d, batch_size=%d, timestep=%d",
    //     traj_x.size(), batch_size, time_steps);

    thrust::device_vector<unsigned char> d_costmap_arr(costmap_arr,
        costmap_arr + costmap_size_x * costmap_size_y);
    thrust::device_vector<float> d_traj_x = traj_x;
    thrust::device_vector<float> d_traj_y = traj_y;
    thrust::device_vector<float> d_traj_yaws = traj_yaws;
    thrust::device_vector<float> d_raw_cost = raw_cost;
    thrust::device_vector<float> d_repulsive_cost = repulsive_cost;

    unsigned char* ptr_costmap_arr = thrust::raw_pointer_cast(d_costmap_arr.data());
    float* ptr_traj_x = thrust::raw_pointer_cast(d_traj_x.data());
    float* ptr_traj_y = thrust::raw_pointer_cast(d_traj_y.data());
    float* ptr_traj_yaws = thrust::raw_pointer_cast(d_traj_yaws.data());
    float* ptr_raw_cost = thrust::raw_pointer_cast(d_raw_cost.data());
    float* ptr_repulsive_cost = thrust::raw_pointer_cast(d_repulsive_cost.data());

    // Launch kernel
    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;

    obstacleKernel<<<numBlocks, blockSize>>>(
        ptr_costmap_arr,
        costmap_size_x,
        costmap_size_y,
        costmap_resolution,
        costmap_origin_x,
        costmap_origin_y,
        ptr_traj_x,
        ptr_traj_y,
        ptr_traj_yaws,
        batch_size,
        time_steps,
        ptr_raw_cost,
        ptr_repulsive_cost,
        N
    );

    cudaDeviceSynchronize();

    // Output
    thrust::host_vector<float> h_raw_cost = d_raw_cost;
    raw_cost = std::vector<float>(h_raw_cost.begin(), h_raw_cost.end());

    thrust::host_vector<float> h_repulsive_cost = d_repulsive_cost;
    repulsive_cost = std::vector<float>(h_repulsive_cost.begin(), h_repulsive_cost.end());
}