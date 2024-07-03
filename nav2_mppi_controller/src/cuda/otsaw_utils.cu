// Copyright (c) 2024 OTSAW

#include "nav2_mppi_controller/cuda/otsaw_utils.cuh"

// "nav2_costmap_2d/cost_values.hpp"
static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
// static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
// static constexpr unsigned char MAX_NON_OBSTACLE = 252;
// static constexpr unsigned char FREE_SPACE = 0;

// costmap_2d.cpp
__device__ bool worldToMap(
    // Input
    double wx,
    double wy,
    // Costmap
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    // Output
    unsigned int & mx,
    unsigned int & my
) {
    if (wx < costmap_origin_x || wy < costmap_origin_y) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - costmap_origin_x) / costmap_resolution);
    my = static_cast<unsigned int>((wy - costmap_origin_y) / costmap_resolution);

    if (mx < costmap_size_x && my < costmap_size_y) {
        return true;
    }
    return false;
}

// footprint_collision_checker.cpp
__device__ double pointCost(
    // Input(0): xy pixel index
    unsigned int x_i,
    unsigned int y_i,
    // Input(1): Costmap
    unsigned char *costmap_arr,
    unsigned int costmap_size_x
) {
    unsigned int index = y_i * costmap_size_x + x_i;
    return costmap_arr[index];
}

// footprint_collision_checker.cpp
__device__ double lineCost(
    // Input(0): 2 points
    int x0, int x1, int y0, int y1,
    // Input(1): Costmap
    unsigned char *costmap_arr,
    unsigned int costmap_size_x
) {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
        point_cost = pointCost(
            line.getX(), line.getY(),
            costmap_arr, costmap_size_x
        );   // Score the current point

        // if in collision, no need to continue
        if (point_cost == static_cast<double>(LETHAL_OBSTACLE)) {
            return point_cost;
        }

        if (line_cost < point_cost) {
            line_cost = point_cost;
        }
    }

    return line_cost;
}

// footprint_collision_checker.cpp
__device__ double footprintCostAtPose(
    // Input(0): Pose
    double x, double y, double theta,
    // Input(1): Costmap
    unsigned char *costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    // Input(2): footprint
    double *footprint_x,
    double *footprint_y,
    unsigned int footprint_size
) {
    double cos_th = cosf(theta);
    double sin_th = sinf(theta);

    double oriented_footprint_x[10];
    double oriented_footprint_y[10];

    for (unsigned int i = 0; i < footprint_size; ++i) {
        double new_x = x + (footprint_x[i] * cos_th - footprint_y[i] * sin_th);
        double new_y = y + (footprint_x[i] * sin_th + footprint_y[i] * cos_th);
        oriented_footprint_x[i] = new_x;
        oriented_footprint_y[i] = new_y;
    }

    // now we really have to lay down the footprint in the costmap_ grid
    unsigned int x0, x1, y0, y1;
    double footprint_cost = 0.0;

    // get the cell coord of the first point
    if (!worldToMap(oriented_footprint_x[0], oriented_footprint_y[0],
        costmap_size_x, costmap_size_y, costmap_resolution,
        costmap_origin_x, costmap_origin_y,
        x0, y0)
    ) {
        return static_cast<double>(LETHAL_OBSTACLE);
    }

    // cache the start to eliminate a worldToMap call
    unsigned int xstart = x0;
    unsigned int ystart = y0;


    // we need to rasterize each line in the footprint
    for (unsigned int i = 0; i < footprint_size - 1; ++i) {
        // get the cell coord of the second point
        if (!worldToMap(oriented_footprint_x[i + 1], oriented_footprint_y[i + 1],
            costmap_size_x, costmap_size_y, costmap_resolution,
            costmap_origin_x, costmap_origin_y,
            x1, y1)
        ) {
            return static_cast<double>(LETHAL_OBSTACLE);
        }

        footprint_cost = fmaxf(
            lineCost(x0, x1, y0, y1, costmap_arr, costmap_size_x),
            footprint_cost);

        // the second point is next iteration's first point
        x0 = x1;
        y0 = y1;

        // if in collision, no need to continue
        if (footprint_cost == static_cast<double>(LETHAL_OBSTACLE)) {
            return footprint_cost;
        }
    }

    // we also need to connect the first point in the footprint to the last point
    // the last iteration's x1, y1 are the last footprint point's coordinates
    return fmaxf(
        lineCost(xstart, x1, ystart, y1, costmap_arr, costmap_size_x),
        footprint_cost);
}

__device__ void costAtPose(
    // Input(0): Pose
    float x,
    float y,
    float theta,
    // Input(1): Costmap
    unsigned char *costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    // Input(2): Footprint
    double *footprint_x,
    double *footprint_y,
    unsigned int footprint_size,
    // Input(3): Config
    bool consider_footprint,
    float possibly_inscribed_cost,
    // Output
    float &cost,
    bool &using_footprint,
    // others
    bool debug = false
) {
    using_footprint = false;
    unsigned int x_i, y_i;

    if (!worldToMap(x, y,
        costmap_size_x, costmap_size_y, costmap_resolution,
        costmap_origin_x, costmap_origin_y,
        x_i, y_i)
    ) {
        cost = NO_INFORMATION;
        return;
    }

    cost = pointCost(x_i, y_i, costmap_arr, costmap_size_x);
    if (debug) {
        printf("[GPU] pointCost(%d, %d)=%.2f\n",
            x_i, y_i, cost);
    }

    if (consider_footprint &&
        (cost >= possibly_inscribed_cost || possibly_inscribed_cost < 1.0f))
    {
        // printf("footprintCostAtPose(%.2f, %.2f, %.2f)", x, y, theta);
        cost = footprintCostAtPose(
            // Input(0): Pose
            x, y, theta,
            // Input(1): Costmap
            costmap_arr,
            costmap_size_x,
            costmap_size_y,
            costmap_resolution,
            costmap_origin_x,
            costmap_origin_y,
            // Input(2): footprint
            footprint_x,
            footprint_y,
            footprint_size
        );
        if (debug) {
            printf("[GPU] footprintCostAtPose=%.2f\n", cost);
        }
        using_footprint = true;
    }
}

__global__ void poseCostKernel(
    // Input(0): Trajectories
    float *traj_x,
    float *traj_y,
    float *traj_yaws,
    unsigned int batch_size,
    unsigned int time_steps,
    // Input(1): Costmap
    unsigned char *costmap_arr,
    unsigned int costmap_size_x,
    unsigned int costmap_size_y,
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    // Input(2): Footprint
    double *footprint_x,
    double *footprint_y,
    unsigned int footprint_size,
    // Input(3): Config
    bool consider_footprint,
    float possibly_inscribed_cost,
    // Output
    float *out_pose_cost,
    bool *out_using_footprint,
    // Iteration limit
    int n
) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < n) {
        // Process
        float cost;
        bool using_footprint;

        costAtPose(
            // Input(0): Pose
            traj_x[tid],
            traj_y[tid],
            traj_yaws[tid],
            // Input(1): Costmap
            costmap_arr,
            costmap_size_x,
            costmap_size_y,
            costmap_resolution,
            costmap_origin_x,
            costmap_origin_y,
            // Input(2): Footprint
            footprint_x,
            footprint_y,
            footprint_size,
            // Input(3): Config
            consider_footprint,
            possibly_inscribed_cost,
            // Output
            cost,
            using_footprint,
            // Debug (i*time_steps+j)
            (tid == 0*time_steps+55)
        );
        
        // batchsize 2000 x timestep 56
        out_pose_cost[tid] = cost;
        out_using_footprint[tid] = using_footprint;

        if (tid == 0*time_steps+55) {
            unsigned int i = tid / time_steps;
            unsigned int j = tid - (i * time_steps);
            
            printf("[GPU] tid=%d, i=%d, j=%d, x=%f, y=%f, yaws=%f, pose_cost=%.2f, using_footprint=%d\n",
                tid,
                i,
                j,
                traj_x[tid],
                traj_y[tid],
                traj_yaws[tid],
                out_pose_cost[tid],
                out_using_footprint[tid] ? 1 : 0
            );
        }
    }
}

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
    double costmap_resolution,
    double costmap_origin_x,
    double costmap_origin_y,
    // Input(2): Footprint
    std::vector<double> footprint_x,
    std::vector<double> footprint_y,
    unsigned int footprint_size,
    // Input(3): Config
    bool consider_footprint,
    float possibly_inscribed_cost,
    // Output:
    std::vector<float>& out_pose_cost,
    std::vector<bool>& out_using_footprint
) {
    // Timing
    // cudaEvent_t t0, t_end;
    // cudaEventCreate(&t0);
    // cudaEventCreate(&t_end);
    // cudaEventRecord(t0);

    // Max number of triggers
    int N = traj_x.size();

    thrust::device_vector<unsigned char> d_vec_costmap_arr(costmap_arr,
        costmap_arr + costmap_size_x * costmap_size_y);
    unsigned char* d_costmap_arr = thrust::raw_pointer_cast(d_vec_costmap_arr.data());

    thrust::device_vector<float> d_vec_traj_x = traj_x;
    float* d_traj_x = thrust::raw_pointer_cast(d_vec_traj_x.data());

    thrust::device_vector<float> d_vec_traj_y = traj_y;
    float* d_traj_y = thrust::raw_pointer_cast(d_vec_traj_y.data());

    thrust::device_vector<float> d_vec_traj_yaws = traj_yaws;
    float* d_traj_yaws = thrust::raw_pointer_cast(d_vec_traj_yaws.data());

    thrust::device_vector<double> d_vec_footprint_x = footprint_x;
    double* d_footprint_x = thrust::raw_pointer_cast(d_vec_footprint_x.data());

    thrust::device_vector<double> d_vec_footprint_y = footprint_y;
    double* d_footprint_y = thrust::raw_pointer_cast(d_vec_footprint_y.data());

    thrust::device_vector<float> d_vec_pose_cost(batch_size);
    float* d_pose_cost = thrust::raw_pointer_cast(d_vec_pose_cost.data());

    thrust::device_vector<bool> d_vec_using_footprint(batch_size);
    bool* d_using_footprint = thrust::raw_pointer_cast(d_vec_using_footprint.data());

    // Launch kernel
    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;

    poseCostKernel<<<numBlocks, blockSize>>>(
        // Input(0): Trajectories
        d_traj_x,
        d_traj_y,
        d_traj_yaws,
        batch_size,
        time_steps,
        // Input(1): Costmap
        d_costmap_arr,
        costmap_size_x,
        costmap_size_y,
        costmap_resolution,
        costmap_origin_x,
        costmap_origin_y,
        // Input(2): Footprint
        d_footprint_x,
        d_footprint_y,
        footprint_size,
        // Input(3): Config
        consider_footprint,
        possibly_inscribed_cost,
        // Output
        d_pose_cost,
        d_using_footprint,
        // Iteration limit
        N
    );

    cudaDeviceSynchronize();

    // Output:
    thrust::host_vector<float> h_pose_cost = d_vec_pose_cost;
    out_pose_cost = std::vector<float>(h_pose_cost.begin(), h_pose_cost.end());

    thrust::host_vector<bool> h_using_footprint = d_vec_using_footprint;
    out_using_footprint = std::vector<bool>(h_using_footprint.begin(), h_using_footprint.end());

    // Calculate elapsed time
    // cudaEventRecord(t_end);
    // cudaEventSynchronize(t_end);
    // float milliseconds = 0;
    // cudaEventElapsedTime(&milliseconds, t0, t_end);
    // std::cout << "t_end-t0: " << milliseconds << " ms" << std::endl;
}