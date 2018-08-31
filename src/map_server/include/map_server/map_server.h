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

#include "map_server/map_reps/map_reps.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "map_server/map_loader.h"

class MappingServerROS
{
public:
	MappingServerROS(const std::string &fname, const std::string &map_type);

private:
	MapLoader *m;
	BaseMapLoader *MyMap;

	// TODO: Add converter for map representations

	rclcpp::Node::SharedPtr n;
};