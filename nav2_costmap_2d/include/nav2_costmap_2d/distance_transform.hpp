// Copyright (c) 2026, Dexory (Tony Najjar)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_COSTMAP_2D__DISTANCE_TRANSFORM_HPP_
#define NAV2_COSTMAP_2D__DISTANCE_TRANSFORM_HPP_

#include <limits>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <Eigen/Core>


namespace nav2_costmap_2d
{

/// Row-major float matrix type for efficient row-wise access
using MatrixXfRM = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/**
 * @class DistanceTransform
 * @brief Efficient Euclidean distance transform using the Felzenszwalb-Huttenlocher algorithm.
 *
 * This class provides a standalone implementation of the linear-time (O(n)) distance transform
 * algorithm based on the lower envelope of parabolas method. It can be used for various
 * applications including costmap inflation, obstacle detection, and path planning.
 *
 * The algorithm computes exact squared Euclidean distances in two separable 1D passes
 * (rows and columns), making it highly efficient for large images/grids.
 *
 * Reference: Distance Transforms of Sampled Functions
 * P. Felzenszwalb and D. Huttenlocher
 * Theory of Computing, Vol. 8, No. 19, September 2012
 */
class DistanceTransform
{
public:
  /// Infinity constant for distance transform
  static constexpr float DT_INF = std::numeric_limits<float>::max();

  /**
   * @brief Perform 1D distance transform using lower envelope of parabolas.
   *
   * This is the core Felzenszwalb-Huttenlocher algorithm for computing squared
   * Euclidean distances in 1D. It uses a lower envelope of parabolas to achieve
   * linear time complexity.
   *
   * @param f Input array of squared distances (typically 0 for obstacles, INF for free space)
   * @param d Output array for transformed squared distances (same size as f)
   * @param n Length of the arrays
   * @param v Buffer for parabola indices (size n)
   * @param z Buffer for parabola boundaries (size n+1)
   */
  static void distanceTransform1D(
    const float * f, float * d, int n,
    int * v, float * z)
  {
    if (!f || !d || !v || !z || n <= 0) {
      return;
    }

    int k = 0;
    v[0] = 0;
    z[0] = -DT_INF;
    z[1] = DT_INF;

    for (int q = 1; q < n; q++) {
      // Use integer arithmetic for squared values to avoid precision loss
      // Only convert to float for the division operation
      float s = (f[q] - f[v[k]] + static_cast<float>(q * q - v[k] * v[k])) /
        (2.0f * static_cast<float>(q - v[k]));
      while (s <= z[k]) {
        k--;
        s = (f[q] - f[v[k]] + static_cast<float>(q * q - v[k] * v[k])) /
          (2.0f * static_cast<float>(q - v[k]));
      }
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = DT_INF;
    }

    k = 0;
    for (int q = 0; q < n; q++) {
      while (z[k + 1] < static_cast<float>(q)) {
        k++;
      }
      const int diff = q - v[k];
      d[q] = static_cast<float>(diff * diff) + f[v[k]];
    }
  }

  /**
   * @brief Perform 2D Euclidean distance transform using separable passes.
   *
   * This method applies the 1D distance transform separately along columns and rows,
   * exploiting the separability property of the Euclidean metric. The result is an
   * exact Euclidean distance map computed in O(width × height) time.
   *
   * The algorithm is parallelized using OpenMP when available for improved performance
   * on multi-core systems.
   *
   * @param img Input/output matrix (modified in place). Input values should be 0 for
   *            obstacles and DT_INF for free space. Output will contain Euclidean distances.
   * @param height Number of rows in the matrix
   * @param width Number of columns in the matrix
   */
  static void distanceTransform2D(MatrixXfRM & img, int height, int width)
  {
    // Column pass (parallelizable)
#ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic, 16)
#endif
    for (int x = 0; x < width; x++) {
      // Thread-local buffers
      std::vector<float> f(height);
      std::vector<float> d(height);
      std::vector<int> v(height);
      std::vector<float> z(height + 1);

      // Extract column
      for (int y = 0; y < height; y++) {
        f[y] = img(y, x);
      }

      // 1D transform
      distanceTransform1D(f.data(), d.data(), height, v.data(), z.data());

      // Write back
      for (int y = 0; y < height; y++) {
        img(y, x) = d[y];
      }
    }

    // Row pass (parallelizable)
#ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic, 16)
#endif
    for (int y = 0; y < height; y++) {
      // Thread-local buffers
      std::vector<float> f(width);
      std::vector<float> d(width);
      std::vector<int> v(width);
      std::vector<float> z(width + 1);

      // Extract row (already contiguous in row-major)
      for (int x = 0; x < width; x++) {
        f[x] = img(y, x);
      }

      // 1D transform
      distanceTransform1D(f.data(), d.data(), width, v.data(), z.data());

      // Write back
      for (int x = 0; x < width; x++) {
        img(y, x) = d[x];
      }
    }

    // Square root to get Euclidean distance (not squared distance)
    img = img.cwiseSqrt();
  }
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DISTANCE_TRANSFORM_HPP_
