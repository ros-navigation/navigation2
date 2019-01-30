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

#ifndef NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
#define NAV2_WORLD_MODEL__WORLD_MODEL_HPP_

#include <string>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "nav2_world_model/world_representation.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace nav2_world_model
{

class WorldModel : public rclcpp::Node
{
public:
  explicit WorldModel(rclcpp::executor::Executor & executor);

private:
  // World representation
  std::map<std::string, std::unique_ptr<WorldRepresentation>> world_representations_;

  // The services provided
  rclcpp::Service<GetCostmap>::SharedPtr get_costmap_service_;
  rclcpp::Service<ProcessRegion>::SharedPtr confirm_free_space_service_;

  void getCostmapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetCostmap::Request> request,
    std::shared_ptr<GetCostmap::Response> response);

  void confirmFreeSpaceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ProcessRegion::Request> request,
    std::shared_ptr<ProcessRegion::Response> response);
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
