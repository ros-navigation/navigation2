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

#ifndef NAV2_MAP_SERVER__BASE_MAP_LOADER_HPP_
#define NAV2_MAP_SERVER__BASE_MAP_LOADER_HPP_

#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>

namespace nav2_map_server
{

class BaseMapLoader
{
public:
  BaseMapLoader() {}
  ~BaseMapLoader() {}

  virtual void LoadMapInfoFromFile(std::string file_name) = 0;
  virtual void LoadMapFromFile(std::string map_name) = 0;
  virtual void PublishMap() = 0;
  virtual void SetMap() = 0;
  virtual void ConnectROS() = 0;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__BASE_MAP_LOADER_HPP_
