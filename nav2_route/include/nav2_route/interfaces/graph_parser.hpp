// Copyright (c) 2023 Joshua Wallace
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

#ifndef NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_INTERFACES_GRAPH_PARSER_HPP_
#define NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_INTERFACES_GRAPH_PARSER_HPP_

#include <string>
#include <memory>

#include "nav2_route/types.hpp"

namespace nav2_route
{

/**
 * @class GraphParser
 * @brief A plugin interface to parse a file into the graph
 */
class GraphParser
{
public:
  using Ptr = std::shared_ptr<GraphParser>;

  /**
   * @brief Constructor
   */
   GraphParser() = default;

   /**
    * @brief Virtual destructor
    */
    virtual ~GraphParser() = default;

    /**
     * @brief Method to load the graph from the filepath
     * @param graph The graph to populate
     * @param filepath The file to parse
     * @return true if graph was successfully loaded
     */
    virtual bool loadGraphFromFile(Graph &graph, const std::string filepath) = 0;
};
}

#endif //NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_INTERFACES_GRAPH_PARSER_HPP_
