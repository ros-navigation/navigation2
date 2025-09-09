// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_ROUTE__NODE_SPATIAL_TREE_HPP_
#define NAV2_ROUTE__NODE_SPATIAL_TREE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <nanoflann.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/types.hpp"
#include "nav2_route/utils.hpp"

namespace nav2_route
{

// Search in XY dimension for nearest neighbors
const size_t DIMENSION = 2;

/**
 * @class nav2_route::GraphAdaptor
 * @brief An adaptor for Nanoflann to operate on our graph object without copying
 */
struct GraphAdaptor
{
  explicit GraphAdaptor(const Graph & obj_)
  : obj(obj_) {}

  inline size_t kdtree_get_point_count() const {return obj.size();}

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const
  {
    if (dim == 0) {
      return obj[idx].coords.x;
    }
    return obj[idx].coords.y;
  }

  template<class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const {return false;}

  const Graph & obj;
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, GraphAdaptor>, GraphAdaptor, DIMENSION,
    unsigned int>
  kd_tree_t;

/**
 * @class nav2_route::NodeSpatialTree
 * @brief An object to find kNNs of the graph to determining the start and end
 * nodes to utilize for planning in a free-space-style request of start and goal poses.
 * Since the graph does not change over time, we can precompute this quadtree and reuse it
 * over many requests.
 */
class NodeSpatialTree
{
public:
  /**
   * @brief Constructor
   */
  NodeSpatialTree() = default;

  /**
   * @brief Destructor
   */
  ~NodeSpatialTree();

  /**
   * @brief Compute the kd-tree based on the graph node information
   * @param graph The graph of nodes for the route
   */
  void computeTree(Graph & graph);

  /**
   * @brief Find the closest node to a given pose
   * @param pose_in Pose to find node near
   * @param node_id The return ID of the node
   * @return if successfully found
   */
  bool findNearestGraphNodesToPose(
    const geometry_msgs::msg::PoseStamped & pose_in,
    std::vector<unsigned int> & node_ids);

  /**
    * @brief Set the number of nodes to search in local area for
    * @param num Numbers of nearest nodes to return
    */
  void setNumOfNearestNodes(int num_of_nearest_nodes);

protected:
  kd_tree_t * kdtree_;
  GraphAdaptor * adaptor_;
  Graph * graph_;
  int num_of_nearest_nodes_{3};
};

}  // namespace nav2_route

#endif  // NAV2_ROUTE__NODE_SPATIAL_TREE_HPP_
