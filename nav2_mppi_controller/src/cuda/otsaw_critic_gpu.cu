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
