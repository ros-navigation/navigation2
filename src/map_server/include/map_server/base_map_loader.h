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

#ifndef MAP_SERVER_BASE_MAP_LOADER_H
#define MAP_SERVER_BASE_MAP_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

class BaseMapLoader
{

public:
	std::string fname;
	std::string mapfname;

	BaseMapLoader() {}

	virtual void loadMapInfoFromFile(std::string fname) = 0;

	virtual void loadMapFromFile(std::string mapfname) = 0;

	virtual void publishMap() = 0;

	virtual void setMap() = 0;

	virtual void connectROS(rclcpp::Node::SharedPtr n) = 0;

	~BaseMapLoader() {}
};

#endif