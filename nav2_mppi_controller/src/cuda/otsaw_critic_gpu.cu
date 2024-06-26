// Copyright (c) 2024 OTSAW

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include "nav2_mppi_controller/cuda/otsaw_critic_gpu.cuh"

#define N 10000000
#define MAX_ERR 1e-6

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, char *file, int line, bool abort=true)
{
    if (code != cudaSuccess) {
        fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}

__global__ void vector_add(float *out, float *a, float *b, int n) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    // Handling arbitrary vector size
    if (tid < n){
        out[tid] = a[tid] + b[tid];
    }
}

float test_gpu_fn() {
    int nDevices;
    cudaGetDeviceCount(&nDevices);
    for (int i = 0; i < nDevices; i++) {
      cudaDeviceProp prop;
      cudaGetDeviceProperties(&prop, i);
      printf("Device Number: %d\n", i);
      printf("  Device name: %s\n", prop.name);
      printf("  Memory Clock Rate (KHz): %d\n",
            prop.memoryClockRate);
      printf("  Memory Bus Width (bits): %d\n",
            prop.memoryBusWidth);
      printf("  Peak Memory Bandwidth (GB/s): %f\n\n",
            2.0*prop.memoryClockRate*(prop.memoryBusWidth/8)/1.0e6);
    }

    float *a, *b, *out;
    float *d_a, *d_b, *d_out; 

    // Allocate host memory
    a   = (float*)malloc(sizeof(float) * N);
    b   = (float*)malloc(sizeof(float) * N);
    out = (float*)malloc(sizeof(float) * N);

    // Initialize host arrays
    for(int i = 0; i < N; i++){
        a[i] = 1.0f;
        b[i] = 2.0f;
    }

    // Allocate device memory 
    gpuErrchk( cudaMalloc((void**)&d_a, sizeof(float) * N));
    gpuErrchk( cudaMalloc((void**)&d_b, sizeof(float) * N));
    gpuErrchk( cudaMalloc((void**)&d_out, sizeof(float) * N));

    // Transfer data from host to device memory
    gpuErrchk( cudaMemcpy(d_a, a, sizeof(float) * N, cudaMemcpyHostToDevice));
    gpuErrchk( cudaMemcpy(d_b, b, sizeof(float) * N, cudaMemcpyHostToDevice));

    // Executing kernel 
    int block_size = 256;
    int grid_size = ((N + block_size) / block_size);
    vector_add<<<grid_size,block_size>>>(d_out, d_a, d_b, N);

    gpuErrchk( cudaDeviceSynchronize() );

    // Transfer data back to host memory
    gpuErrchk( cudaMemcpy(out, d_out, sizeof(float) * N, cudaMemcpyDeviceToHost));

    // Verification
    printf("a[0]=%f\n", a[0]);
    printf("b[0]=%f\n", b[0]);
    printf("out[0]=%f\n", out[0]);
    float ret = out[0];

    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk( cudaDeviceSynchronize() );

    // Deallocate device memory
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_out);

    // Deallocate host memory
    free(a); 
    free(b); 
    free(out);

    return ret;
}
