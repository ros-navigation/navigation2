// Copyright (c) 2025, Open Navigation LLC
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

#include <string>
#include <memory>
#include <vector>

#include "nav2_route/node_spatial_tree.hpp"

namespace nav2_route
{

NodeSpatialTree::~NodeSpatialTree()
{
  if (kdtree_) {
    delete kdtree_;
    kdtree_ = nullptr;
  }

  if (adaptor_) {
    delete adaptor_;
    adaptor_ = nullptr;
  }
}

void NodeSpatialTree::setNumOfNearestNodes(int num_of_nearest_nodes)
{
  num_of_nearest_nodes_ = num_of_nearest_nodes;
}

void NodeSpatialTree::computeTree(Graph & graph)
{
  if (kdtree_) {
    delete kdtree_;
    kdtree_ = nullptr;
  }

  if (adaptor_) {
    delete adaptor_;
    adaptor_ = nullptr;
  }

  adaptor_ = new GraphAdaptor(graph);
  kdtree_ = new kd_tree_t(DIMENSION, *adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_->buildIndex();
  graph_ = &graph;
}

bool NodeSpatialTree::findNearestGraphNodesToPose(
  const geometry_msgs::msg::PoseStamped & pose_in, std::vector<unsigned int> & node_ids)
{
  size_t num_results = static_cast<size_t>(num_of_nearest_nodes_);
  std::vector<unsigned int> ret_index(num_results);
  std::vector<double> out_dist_sqr(num_results);
  const double query_pt[2] = {pose_in.pose.position.x, pose_in.pose.position.y};
  num_results = kdtree_->knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  if (num_results == 0) {
    return false;
  }

  for (int i = 0; i < static_cast<int>(ret_index.size()); ++i) {
    node_ids.push_back(ret_index[i]);
  }
  return true;
}

}  // namespace nav2_route
