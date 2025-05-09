// Copyright (c) 2025 Leander Stephen Desouza
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NAV2_COSTMAP_2D__COSTMAP_UTILS_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_UTILS_HPP_

#include <string>
#include <memory>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_costmap_2d
{
/**
* Joins the specified topic with the parent namespace of the costmap node.
* If the topic has an absolute path, it is returned instead.
*
* This is necessary for user defined relative topics to work as expected since costmap layers
* add an additional `costmap_name` namespace to the topic.
* For example:
*   * User chosen namespace is `tb4`.
*   * User chosen topic is `scan`.
*   * Costmap node namespace will be `/tb4/global_costmap`.
*   * Without this function, the topic would be `/tb4/global_costmap/scan`.
*   * With this function, topic will be remapped to `/tb4/scan`.
* Use global topic `/scan` if you do not wish the node namespace to apply.
*/
inline std::string joinWithParentNamespace(
  const std::weak_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::string & topic)
{
  auto node_ptr = node.lock();
  if (!node_ptr) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (topic[0] != '/') {
    std::string node_namespace = node_ptr->get_namespace();
    std::string parent_namespace = node_namespace.substr(0, node_namespace.rfind("/"));
    return parent_namespace + "/" + topic;
  }

  return topic;
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_UTILS_HPP_
