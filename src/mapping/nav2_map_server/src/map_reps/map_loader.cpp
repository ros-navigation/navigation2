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

#include "map_server/map_loader.h"

// Factory Class for loading new map types
BaseMapLoader *MapLoader::createMap(std::string mapType, rclcpp::Node::SharedPtr n, std::string filename)
{
	if (mapType == "occupancy")
	{
		return new OccGridLoader(n, filename);
	}
	else if (mapType == "gridmap")
	{
		return new OccGridLoader(n, filename); // TODO: Port grid_map, substitute with GridMapLoader
	}
	else
	{
		fprintf(stderr, "[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", mapType.c_str());
		exit(-1);
	}
}

BaseMapLoader *MapLoader::createMap(std::string mapType)
{
	if (mapType == "occupancy")
	{
		return new OccGridLoader;
	}
	else if (mapType == "gridmap")
	{
		return new OccGridLoader; // TODO: Port grid_map, substitute with GridMapLoader
	}
	else
	{
		fprintf(stderr, "[ERROR] [map_server]: Cannot Load Map of Type '%s'\n", mapType.c_str());
		exit(-1);
	}
}