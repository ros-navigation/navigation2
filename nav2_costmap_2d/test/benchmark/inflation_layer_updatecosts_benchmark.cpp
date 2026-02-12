// Copyright (c) 2026
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

#include <sys/stat.h>
#include <sys/types.h>
#include <random>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <set>

#include "benchmark/benchmark.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{
static constexpr const char * global_frame{"map"};

/**
 * @brief Test wrapper for InflationLayer to expose protected methods
 */
class TestInflationLayer : public nav2_costmap_2d::InflationLayer
{
public:
  void setupForBenchmark(
    nav2_costmap_2d::LayeredCostmap & layers,
    nav2::LifecycleNode::SharedPtr node,
    double inflation_radius,
    double cost_scaling_factor)
  {
    // Declare parameters before initialization so the layer reads them
    node->declare_parameter("inflation.inflation_radius", rclcpp::ParameterValue(inflation_radius));
    node->declare_parameter("inflation.cost_scaling_factor",
        rclcpp::ParameterValue(cost_scaling_factor));
    node->declare_parameter("inflation.inflate_unknown", rclcpp::ParameterValue(false));
    node->declare_parameter("inflation.inflate_around_unknown", rclcpp::ParameterValue(false));

    initialize(&layers, "inflation", nullptr, node, nullptr);
  }

  // Expose updateCosts for direct benchmarking
  void benchmarkUpdateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
  {
    updateCosts(master_grid, min_i, min_j, max_i, max_j);
  }
};

/**
 * @brief Configuration for benchmark scenarios
 */
struct BenchmarkConfig
{
  unsigned int width;        // Costmap width in cells
  unsigned int height;       // Costmap height in cells
  double resolution;         // Meters per cell
  double occupancy;          // Percentage of cells occupied (0.0 to 1.0)
  double inflation_radius;   // Inflation radius in meters
  std::string description;   // Human-readable description
};

/**
 * @brief Generate obstacles in a costmap based on occupancy percentage
 */
[[maybe_unused]]
static void generateObstacles(
  nav2_costmap_2d::Costmap2D & costmap,
  double occupancy_percent,
  unsigned int seed = 42)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const unsigned int total_cells = size_x * size_y;
  const unsigned int num_obstacles = static_cast<unsigned int>(total_cells * occupancy_percent);

  std::mt19937 gen(seed);
  std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
  std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);

  // First, clear the costmap
  unsigned char * master_array = costmap.getCharMap();
  memset(master_array, nav2_costmap_2d::FREE_SPACE, total_cells);

  // Add obstacles randomly
  for (unsigned int i = 0; i < num_obstacles; ++i) {
    unsigned int x = dist_x(gen);
    unsigned int y = dist_y(gen);
    costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

/**
 * @brief Generate clustered obstacles (more realistic)
 */
[[maybe_unused]]
void generateClusteredObstacles(
  nav2_costmap_2d::Costmap2D & costmap,
  double occupancy_percent,
  unsigned int num_clusters = 10,
  unsigned int seed = 42)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const unsigned int total_cells = size_x * size_y;
  const unsigned int num_obstacles = static_cast<unsigned int>(total_cells * occupancy_percent);

  std::mt19937 gen(seed);
  std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
  std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);
  std::normal_distribution<double> offset_dist(0.0, 5.0);

  // First, clear the costmap
  unsigned char * master_array = costmap.getCharMap();
  memset(master_array, nav2_costmap_2d::FREE_SPACE, total_cells);

  // Generate cluster centers
  std::vector<std::pair<unsigned int, unsigned int>> cluster_centers;
  for (unsigned int i = 0; i < num_clusters; ++i) {
    cluster_centers.emplace_back(dist_x(gen), dist_y(gen));
  }

  // Distribute obstacles around clusters
  std::uniform_int_distribution<size_t> cluster_dist(0, num_clusters - 1);
  for (unsigned int i = 0; i < num_obstacles; ++i) {
    size_t cluster_idx = cluster_dist(gen);
    int x = static_cast<int>(cluster_centers[cluster_idx].first) +
      static_cast<int>(offset_dist(gen));
    int y = static_cast<int>(cluster_centers[cluster_idx].second) +
      static_cast<int>(offset_dist(gen));

    // Clamp to valid range
    x = std::max(0, std::min(static_cast<int>(size_x - 1), x));
    y = std::max(0, std::min(static_cast<int>(size_y - 1), y));

    costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

/**
 * @brief Generate rectangular obstacles until occupancy ratio is satisfied
 */
void generateRectangularObstacles(
  nav2_costmap_2d::Costmap2D & costmap,
  double occupancy_percent,
  double min_rect_width_pct = 0.05,   // Min 5% of map width
  double max_rect_width_pct = 0.15,   // Max 15% of map width
  double min_rect_height_pct = 0.05,  // Min 5% of map height
  double max_rect_height_pct = 0.15,  // Max 15% of map height
  unsigned int seed = 42)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const unsigned int total_cells = size_x * size_y;
  const unsigned int target_occupied_cells = static_cast<unsigned int>(total_cells *
    occupancy_percent);

  // Calculate proportional rectangle sizes based on map dimensions
  const unsigned int min_rect_width = std::max(1u,
      static_cast<unsigned int>(size_x * min_rect_width_pct));
  const unsigned int max_rect_width = std::max(min_rect_width + 1,
      static_cast<unsigned int>(size_x * max_rect_width_pct));
  const unsigned int min_rect_height = std::max(1u,
      static_cast<unsigned int>(size_y * min_rect_height_pct));
  const unsigned int max_rect_height = std::max(min_rect_height + 1,
      static_cast<unsigned int>(size_y * max_rect_height_pct));

  std::mt19937 gen(seed);
  std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
  std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);
  std::uniform_int_distribution<unsigned int> dist_width(min_rect_width, max_rect_width);
  std::uniform_int_distribution<unsigned int> dist_height(min_rect_height, max_rect_height);

  // First, clear the costmap
  unsigned char * master_array = costmap.getCharMap();
  memset(master_array, nav2_costmap_2d::FREE_SPACE, total_cells);

  // Track occupied cells
  unsigned int occupied_count = 0;

  // Place rectangles until we reach target occupancy
  const unsigned int max_attempts = 10000;  // Prevent infinite loop
  unsigned int attempts = 0;

  while (occupied_count < target_occupied_cells && attempts < max_attempts) {
    // Generate random rectangle
    unsigned int rect_x = dist_x(gen);
    unsigned int rect_y = dist_y(gen);
    unsigned int rect_width = dist_width(gen);
    unsigned int rect_height = dist_height(gen);

    // Clamp rectangle to fit within costmap
    unsigned int end_x = std::min(rect_x + rect_width, size_x);
    unsigned int end_y = std::min(rect_y + rect_height, size_y);

    // Fill rectangle with obstacles and count new occupied cells
    for (unsigned int y = rect_y; y < end_y; ++y) {
      for (unsigned int x = rect_x; x < end_x; ++x) {
        unsigned char current_cost = costmap.getCost(x, y);
        if (current_cost != nav2_costmap_2d::LETHAL_OBSTACLE) {
          costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
          occupied_count++;

          // Early exit if we've reached target
          if (occupied_count >= target_occupied_cells) {
            return;
          }
        }
      }
    }

    attempts++;
  }
}

/**
 * @brief Create a robot footprint (rectangular)
 * @param length Length of robot in meters
 * @param width Width of robot in meters
 */
std::vector<geometry_msgs::msg::Point> createRectangularFootprint(
  double length = 0.6, double width = 0.4)
{
  std::vector<geometry_msgs::msg::Point> footprint;

  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = length / 2.0;  p1.y = width / 2.0;
  p2.x = length / 2.0;  p2.y = -width / 2.0;
  p3.x = -length / 2.0; p3.y = -width / 2.0;
  p4.x = -length / 2.0; p4.y = width / 2.0;

  footprint.push_back(p1);
  footprint.push_back(p2);
  footprint.push_back(p3);
  footprint.push_back(p4);

  return footprint;
}

/**
 * @brief Create directory recursively if it doesn't exist
 */
bool createDirectory(const std::string & path)
{
  struct stat st;
  if (stat(path.c_str(), &st) == 0) {
    return S_ISDIR(st.st_mode);
  }

  // Try to create directory
  if (mkdir(path.c_str(), 0755) == 0) {
    return true;
  }

  // If failed, try creating parent directory first
  size_t pos = path.find_last_of('/');
  if (pos != std::string::npos) {
    if (createDirectory(path.substr(0, pos))) {
      return mkdir(path.c_str(), 0755) == 0;
    }
  }

  return false;
}

/**
 * @brief Save costmap as grayscale PGM image
 */
bool saveCostmapAsPGM(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::string & filename)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    return false;
  }

  unsigned int width = costmap.getSizeInCellsX();
  unsigned int height = costmap.getSizeInCellsY();
  const unsigned char * data = costmap.getCharMap();

  // PGM header
  file << "P5\n";
  file << width << " " << height << "\n";
  file << "255\n";

  // Write pixel data (inverted: 255=free, 0=lethal)
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      unsigned char cost = data[y * width + x];
      unsigned char pixel = 255 - cost;  // Invert so obstacles are dark
      file.put(pixel);
    }
  }

  return file.good();
}

/**
 * @brief Save costmap as color PPM image with inflation visualization
 */
bool saveCostmapAsColorPPM(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::string & filename)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    return false;
  }

  unsigned int width = costmap.getSizeInCellsX();
  unsigned int height = costmap.getSizeInCellsY();
  const unsigned char * data = costmap.getCharMap();

  // PPM header
  file << "P6\n";
  file << width << " " << height << "\n";
  file << "255\n";

  // Write RGB pixel data
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      unsigned char cost = data[y * width + x];
      unsigned char r = 255, g = 255, b = 255;  // Default: white (free space)

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        // Lethal: black
        r = g = b = 0;
      } else if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Inscribed: red
        r = 255; g = 0; b = 0;
      } else if (cost > nav2_costmap_2d::FREE_SPACE &&  // NOLINT(readability/braces)
        cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        // Inflated costs: gradient from blue (low) to yellow (high)
        float normalized = static_cast<float>(cost) /
          static_cast<float>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        if (normalized < 0.5f) {
          // Blue to cyan (0.0 - 0.5)
          float t = normalized * 2.0f;
          r = 0;
          g = static_cast<unsigned char>(255 * t);
          b = 255;
        } else {
          // Cyan to yellow (0.5 - 1.0)
          float t = (normalized - 0.5f) * 2.0f;
          r = static_cast<unsigned char>(255 * t);
          g = 255;
          b = static_cast<unsigned char>(255 * (1.0f - t));
        }
      }

      file.put(r);
      file.put(g);
      file.put(b);
    }
  }

  return file.good();
}

}  // namespace

/**
 * @brief Fixture for inflation layer benchmarks
 */
class InflationLayerFixture : public benchmark::Fixture
{
public:
  static bool save_images_enabled;
  static std::string & output_directory()
  {
    static std::string dir = "benchmark_results";
    return dir;
  }

  void SetUp(benchmark::State & state) override
  {
    // Get parameters from benchmark state
    width_ = state.range(0);
    height_ = state.range(1);
    occupancy_ = state.range(2) / 100.0;  // Convert percentage to decimal
    inflation_radius_ = state.range(3) / 100.0;  // Convert cm to meters
    resolution_ = state.range(4) / 100.0;  // Convert cm to meters
    cost_scaling_factor_ = state.range(5) / 10.0;  // Convert 10x scaled to actual

    // Initialize ROS node
    if (!node_) {
      node_ = std::make_shared<nav2::LifecycleNode>("inflation_benchmark_node");
    }

    // Setup layered costmap
    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(global_frame, false, false);
    layers_->resizeMap(width_, height_, resolution_, 0.0, 0.0);

    // Set robot footprint (1.2m x 1.5m rectangular robot)
    auto footprint = createRectangularFootprint(1.2, 1.5);
    layers_->setFootprint(footprint);

    // Create and setup inflation layer
    inflation_layer_ = std::make_shared<TestInflationLayer>();
    inflation_layer_->setupForBenchmark(*layers_, node_, inflation_radius_, cost_scaling_factor_);

    // Get the master costmap
    master_costmap_ = layers_->getCostmap();

    // Generate obstacles based on occupancy (using rectangular obstacles)
    generateRectangularObstacles(*master_costmap_, occupancy_);
  }

  void TearDown(benchmark::State & state) override
  {
    // Save images if enabled (only on the last iteration to avoid spam)
    if (save_images_enabled && master_costmap_ && state.thread_index() == 0) {
      static std::set<std::string> saved_files;

      // Create output directory if it doesn't exist
      createDirectory(output_directory());

      // Generate filename based on parameters
      std::ostringstream filename;
      filename << output_directory() << "/inflation_"
               << width_ << "x" << height_
               << "_occ" << static_cast<int>(occupancy_ * 100)
               << "_rad" << static_cast<int>(inflation_radius_ * 100)
               << ".ppm";

      // Only save and report once per unique configuration
      if (saved_files.find(filename.str()) == saved_files.end()) {
        if (saveCostmapAsColorPPM(*master_costmap_, filename.str())) {
          std::cout << "  Saved: " << filename.str() << std::endl;
          saved_files.insert(filename.str());
        }
      }
    }

    inflation_layer_.reset();
    layers_.reset();
    node_.reset();  // Reset node so parameters can be declared fresh next time
  }

  unsigned int width_;
  unsigned int height_;
  double occupancy_;
  double resolution_;
  double inflation_radius_;
  double cost_scaling_factor_;
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestInflationLayer> inflation_layer_;
  nav2_costmap_2d::Costmap2D * master_costmap_;
};

// Initialize static members
bool InflationLayerFixture::save_images_enabled = false;

/**
 * @brief Benchmark the updateCosts function with various configurations
 */
BENCHMARK_DEFINE_F(InflationLayerFixture, UpdateCosts)(benchmark::State & state)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_;
  const int max_j = height_;

  for (auto _ : state) {
    // Benchmark the updateCosts function
    inflation_layer_->benchmarkUpdateCosts(*master_costmap_, min_i, min_j, max_i, max_j);
  }

  // Report additional metrics
  const size_t total_cells = width_ * height_;
  const size_t occupied_cells = static_cast<size_t>(total_cells * occupancy_);

  state.counters["cells"] = total_cells;
  state.counters["occupied"] = occupied_cells;
  state.counters["occupancy_%"] = occupancy_ * 100.0;
  state.counters["inflation_r"] = inflation_radius_;
  state.counters["resolution"] = resolution_;
  state.counters["cost_scale"] = cost_scaling_factor_;
  state.counters["width"] = width_;
  state.counters["height"] = height_;
  state.counters["cells/s"] = benchmark::Counter(
    total_cells, benchmark::Counter::kIsIterationInvariantRate);
}

// Predefined scenarios - Various inflation radii and occupancy levels
// Arguments: width, height, occupancy_%, inflation_radius_cm, resolution_cm, cost_scaling_10x

// ========================================
// Resolution Impact (same physical area)
// ========================================
// 100x100m area at different resolutions
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({1000, 1000, 50, 200, 10, 30})     // 100x100m @ 0.1m res, 2m radius
->Args({2000, 2000, 50, 200, 5, 30})      // 100x100m @ 0.05m res, 2m radius
->Args({3333, 3333, 50, 200, 3, 30})      // 100x100m @ 0.03m res, 2m radius
->Unit(benchmark::kMillisecond);

// ========================================
// Occupancy Variations (realistic scenarios)
// ========================================
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({1500, 1500, 10, 200, 5, 30})      // Sparse (10%) - open warehouse
->Args({1500, 1500, 30, 200, 5, 30})      // Light (30%) - typical warehouse
->Args({1500, 1500, 50, 200, 5, 30})      // Medium (50%) - busy area
->Args({1500, 1500, 80, 200, 5, 30})      // Dense (80%) - cluttered environment
->Unit(benchmark::kMillisecond);

// ========================================
// Inflation Radius Extremes
// ========================================
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({1000, 1000, 50, 50, 5, 30})       // 0.5m - tight corridors
->Args({1000, 1000, 50, 100, 5, 30})      // 1m - standard robot
->Args({1000, 1000, 50, 200, 5, 30})      // 2m - medium safety margin
->Args({1000, 1000, 50, 300, 5, 30})      // 3m - large safety margin
->Args({1000, 1000, 50, 500, 5, 30})      // 5m - aggressive safety
->Args({1000, 1000, 50, 1000, 5, 30})     // 10m - extreme case
->Unit(benchmark::kMillisecond);

// ========================================
// Cost Scaling Factor Impact
// ========================================
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({1000, 1000, 50, 200, 5, 10})      // cost_scale = 1.0 (slower decay)
->Args({1000, 1000, 50, 200, 5, 30})      // cost_scale = 3.0 (default)
->Args({1000, 1000, 50, 200, 5, 50})      // cost_scale = 5.0 (faster decay)
->Args({1000, 1000, 50, 200, 5, 100})     // cost_scale = 10.0 (very fast decay)
->Unit(benchmark::kMillisecond);

// ========================================
// Real-World Production Scenarios
// ========================================
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({2000, 2000, 30, 100, 5, 30})      // 100x100m warehouse, 1m radius
->Args({4000, 4000, 30, 100, 5, 30})      // 200x200m warehouse, 1m radius
->Args({1500, 1000, 40, 150, 5, 30})      // 75x50m factory floor, 1.5m radius
->Args({3000, 3000, 20, 200, 5, 30})      // 150x150m open area, 2m radius
->Unit(benchmark::kMillisecond);

// ========================================
// Small Maps (Quick Tests)
// ========================================
BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
->Args({300, 300, 50, 55, 5, 30})         // Small test map
->Args({500, 500, 50, 100, 5, 30})        // Medium test map
->Unit(benchmark::kMillisecond);

// ========================================
// Large Maps (Stress Tests) - Commented out by default
// ========================================
// BENCHMARK_REGISTER_F(InflationLayerFixture, UpdateCosts)
//   ->Args({8000, 8000, 50, 200, 5, 30})   // 400x400m @ 0.05m, 2m radius
//   ->Args({10000, 10000, 30, 300, 5, 30}) // 500x500m @ 0.05m, 3m radius
//   ->Unit(benchmark::kMillisecond);

/**
 * @brief Custom benchmark that allows command-line specification
 */
class CustomInflationBenchmark
{
public:
  static void run(
    unsigned int width, unsigned int height,
    double occupancy, double inflation_radius = 0.55,
    double cost_scaling_factor = 3.0, const std::string & visualize_path = "")
  {
    auto node = std::make_shared<nav2::LifecycleNode>("custom_benchmark_node");

    nav2_costmap_2d::LayeredCostmap layers(global_frame, false, false);
    layers.resizeMap(width, height, 0.05, 0.0, 0.0);

    // Set robot footprint (1.2m x 1.5m rectangular robot)
    auto footprint = createRectangularFootprint(1.2, 1.5);
    layers.setFootprint(footprint);

    auto inflation_layer = std::make_shared<TestInflationLayer>();
    inflation_layer->setupForBenchmark(layers, node, inflation_radius, cost_scaling_factor);

    auto master_costmap = layers.getCostmap();
    generateRectangularObstacles(*master_costmap, occupancy);

    // Warm-up run
    inflation_layer->benchmarkUpdateCosts(*master_costmap, 0, 0, width, height);

    // Timed runs
    const int num_iterations = 5;
    std::vector<double> times;
    times.reserve(num_iterations);

    for (int i = 0; i < num_iterations; ++i) {
      auto start = std::chrono::high_resolution_clock::now();
      inflation_layer->benchmarkUpdateCosts(*master_costmap, 0, 0, width, height);
      auto end = std::chrono::high_resolution_clock::now();

      std::chrono::duration<double, std::milli> duration = end - start;
      times.push_back(duration.count());
    }

    // Calculate statistics
    double sum = 0.0;
    double min_time = times[0];
    double max_time = times[0];

    for (double t : times) {
      sum += t;
      min_time = std::min(min_time, t);
      max_time = std::max(max_time, t);
    }

    double mean = sum / num_iterations;

    double variance = 0.0;
    for (double t : times) {
      variance += (t - mean) * (t - mean);
    }
    variance /= num_iterations;
    double stddev = std::sqrt(variance);

    // Print results
    std::cout << "\n========================================\n";
    std::cout << "Custom Inflation Layer Benchmark Results\n";
    std::cout << "========================================\n";
    std::cout << "Configuration:\n";
    std::cout << "  Map size: " << width << " x " << height << " cells\n";
    std::cout << "  Total cells: " << (width * height) << "\n";
    std::cout << "  Occupancy: " << (occupancy * 100.0) << "%\n";
    std::cout << "  Robot footprint: 1.2m x 1.5m rectangular\n";
    std::cout << "  Inscribed radius: " << layers.getInscribedRadius() << " m\n";
    std::cout << "  Inflation radius: " << inflation_radius << " m\n";
    std::cout << "  Cost scaling factor: " << cost_scaling_factor << "\n";
    std::cout << "  OpenMP: ";
#ifdef _OPENMP
    std::cout << "ENABLED (max threads: " << omp_get_max_threads() << ")\n";
#else
    std::cout << "DISABLED\n";
#endif
    std::cout << "  Iterations: " << num_iterations << "\n";
    std::cout << "\nTiming Results:\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Mean: " << mean << " ms\n";
    std::cout << "  Std Dev: " << stddev << " ms\n";
    std::cout << "  Min: " << min_time << " ms\n";
    std::cout << "  Max: " << max_time << " ms\n";
    std::cout << "\nPerformance Metrics:\n";
    std::cout << "  Cells/second: " << std::fixed << std::setprecision(0)
              << ((width * height) / (mean / 1000.0)) << "\n";
    std::cout << "  Throughput: " << std::fixed << std::setprecision(2)
              << (1000.0 / mean) << " updates/second\n";
    std::cout << "========================================\n\n";

    // Save visualization if requested
    if (!visualize_path.empty()) {
      std::string pgm_path = visualize_path;
      std::string ppm_path = visualize_path;

      // Ensure proper extensions
      if (pgm_path.size() < 4 || pgm_path.substr(pgm_path.size() - 4) != ".pgm") {
        pgm_path += ".pgm";
      }
      ppm_path = pgm_path.substr(0, pgm_path.size() - 4) + ".ppm";

      std::cout << "Saving visualizations...\n";
      if (saveCostmapAsPGM(*master_costmap, pgm_path)) {
        std::cout << "  Grayscale PGM saved to: " << pgm_path << "\n";
      } else {
        std::cout << "  Failed to save PGM\n";
      }

      if (saveCostmapAsColorPPM(*master_costmap, ppm_path)) {
        std::cout << "  Color PPM saved to: " << ppm_path << "\n";
      } else {
        std::cout << "  Failed to save PPM\n";
      }
    }
  }
};

void printUsage()
{
  std::cout << "\nInflation Layer UpdateCosts Benchmark\n";
  std::cout << "======================================\n\n";
  std::cout << "Usage:\n";
  std::cout << "  Run predefined scenarios:\n";
  std::cout << "    ./inflation_layer_updatecosts_benchmark\n\n";
  std::cout << "  Run custom benchmark:\n";
  std::cout << "    ./inflation_layer_updatecosts_benchmark --custom --width=<W> "
    "--height=<H> --occupancy=<O>\n\n";
  std::cout << "Options:\n";
  std::cout << "  --custom              Run custom benchmark instead of Google Benchmark suite\n";
  std::cout << "  --width=<N>           Map width in cells (default: 1000)\n";
  std::cout << "  --height=<N>          Map height in cells (default: 1000)\n";
  std::cout << "  --occupancy=<N>       Obstacle occupancy percentage 0-100 (default: 10)\n";
  std::cout << "  --inflation=<N>       Inflation radius in meters (default: 0.55)\n";
  std::cout << "  --cost_scaling=<N>    Cost scaling factor (default: 3.0)\n";
  std::cout << "  --visualize=<path>    Save costmap as PGM/PPM images (custom mode only)\n";
  std::cout << "  --save_images         Save result images for each benchmark (default: off)\n";
  std::cout <<
    "  --output_dir=<path>   Output directory for images (default: benchmark_results)\n\n";
  std::cout << "Google Benchmark Options:\n";
  std::cout << "  --benchmark_filter=<regex>     Run only benchmarks matching the regex\n";
  std::cout << "  --benchmark_min_time=<N>       Minimum time in seconds to run each benchmark\n";
  std::cout << "  --benchmark_repetitions=<N>    Number of times to repeat each benchmark\n";
  std::cout << "  --benchmark_format=<console|json|csv>\n";
  std::cout << "  --benchmark_out=<filename>     Output file for benchmark results\n";
  std::cout << "  --help                         Show Google Benchmark help\n\n";
  std::cout << "Examples:\n";
  std::cout << "  # Run all predefined scenarios\n";
  std::cout << "  ./inflation_layer_updatecosts_benchmark\n\n";
  std::cout << "  # Run custom 2000x2000 map with 15% occupancy\n";
  std::cout << "  ./inflation_layer_updatecosts_benchmark --custom --width=2000 "
    "--height=2000 --occupancy=15\n\n";
  std::cout << "  # Run only medium-sized benchmarks\n";
  std::cout << "  ./inflation_layer_updatecosts_benchmark --benchmark_filter=500x500\n\n";
  std::cout << "  # Run with multiple repetitions and save to CSV\n";
  std::cout << "  ./inflation_layer_updatecosts_benchmark --benchmark_repetitions=10 "
    "--benchmark_format=csv --benchmark_out=results.csv\n\n";
}

int main(int argc, char ** argv)
{
  // Check for custom benchmark mode
  bool custom_mode = false;
  unsigned int custom_width = 1000;
  unsigned int custom_height = 1000;
  double custom_occupancy = 0.10;
  double custom_inflation = 0.55;
  double custom_cost_scaling = 3.0;
  std::string visualize_path;

  // Build filtered argv for Google Benchmark (exclude our custom flags)
  std::vector<char *> filtered_argv;
  filtered_argv.push_back(argv[0]);  // Keep program name

  // Parse custom arguments and filter out custom flags
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--custom") {
      custom_mode = true;
    } else if (arg.find("--width=") == 0) {
      custom_width = std::stoul(arg.substr(8));
    } else if (arg.find("--height=") == 0) {
      custom_height = std::stoul(arg.substr(9));
    } else if (arg.find("--occupancy=") == 0) {
      custom_occupancy = std::stod(arg.substr(12)) / 100.0;
    } else if (arg.find("--inflation=") == 0) {
      custom_inflation = std::stod(arg.substr(12));
    } else if (arg.find("--cost_scaling=") == 0) {
      custom_cost_scaling = std::stod(arg.substr(15));
    } else if (arg.find("--visualize=") == 0) {
      visualize_path = arg.substr(12);
    } else if (arg == "--save_images") {
      InflationLayerFixture::save_images_enabled = true;
    } else if (arg.find("--output_dir=") == 0) {
      InflationLayerFixture::output_directory() = arg.substr(13);
    } else if (arg == "--usage") {
      printUsage();
      return 0;
    } else {
      // Pass through to Google Benchmark
      filtered_argv.push_back(argv[i]);
    }
  }

  // Initialize ROS
  rclcpp::init(argc, argv);

  // Suppress ROS logging noise during benchmarks (set to ERROR level)
  auto ret = rcutils_logging_set_logger_level(
    "inflation_benchmark_node", RCUTILS_LOG_SEVERITY_ERROR);
  if (ret != RCUTILS_RET_OK) {
    std::cerr << "Failed to set logger level\n";
  }
  ret = rcutils_logging_set_logger_level(
    "custom_benchmark_node", RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level(
    "rclcpp_lifecycle", RCUTILS_LOG_SEVERITY_ERROR);

  // Print OpenMP status
#ifdef _OPENMP
  std::cout << "OpenMP: ENABLED" << std::endl;
#else
  std::cout << "OpenMP: DISABLED" << std::endl;
#endif

  if (custom_mode) {
    // Run custom benchmark
    CustomInflationBenchmark::run(custom_width, custom_height, custom_occupancy, custom_inflation,
      custom_cost_scaling, visualize_path);
  } else {
    // Print image saving status
    if (InflationLayerFixture::save_images_enabled) {
      std::cout << "Image saving enabled. Output directory: "
                << InflationLayerFixture::output_directory() << std::endl;
    }

    // Run Google Benchmark suite with filtered arguments
    int filtered_argc = static_cast<int>(filtered_argv.size());
    benchmark::Initialize(&filtered_argc, filtered_argv.data());
    if (benchmark::ReportUnrecognizedArguments(filtered_argc, filtered_argv.data())) {
      printUsage();
      rclcpp::shutdown();
      return 1;
    }
    benchmark::RunSpecifiedBenchmarks();
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
