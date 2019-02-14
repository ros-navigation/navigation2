// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__LIFECYCLE_UTILS_HPP_
#define NAV2_UTIL__LIFECYCLE_UTILS_HPP_

#include <vector>
#include <string>
#include "nav2_util/string_utils.hpp"

namespace nav2_util
{

/// Transition the given lifecycle nodes to the ACTIVATED state in order
void bringupLifecycleNodes(const std::vector<std::string> & node_names);

/// Transition the given lifecycle nodes to the ACTIVATED state in order.
/**
 * \param[in] nodes A ':' seperated list of node names. eg. "/node1:/node2"
 */
void bringupLifecycleNodes(const std::string & nodes)
{
  bringupLifecycleNodes(split(nodes, ':'));
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__LIFECYCLE_UTILS_HPP_
