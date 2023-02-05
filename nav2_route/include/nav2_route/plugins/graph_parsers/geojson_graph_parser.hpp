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



#include <memory>
#include <string>
#include <nlohmann/json.hpp>

#include "nav2_route/interfaces/graph_parser.hpp"

#ifndef NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_PLUGINS_GRAPH_PARSERS_GEOJSON_GRAPH_PARSER_HPP_
#define NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_PLUGINS_GRAPH_PARSERS_GEOJSON_GRAPH_PARSER_HPP_

namespace nav2_route
{

class GeoJsonGraphParser : public GraphParser
{
public:
  using Json = nlohmann::json;

  GeoJsonGraphParser() = default;

  ~GeoJsonGraphParser() = default;

  bool loadGraphFromFile(Graph &graph, const std::string filepath) override;

private:

  /**
   * @brief Get all nodes from the features
   * @param features The features to grab the graph nodes from
   * @param[out] nodes The nodes found within the features
   */
  void getNodes(const Json & features, std::vector<Json> & nodes);

  /**
   * @brief Get all edges from the features
   * @param features The features to grab the graph edges from
   * @param edges The edges found with the features
   */
  void getEdges(const Json & features, std::vector<Json> & edges);

  /**
   * @brief Add all nodes into the graph
   * @param graph The graph in which the nodes are added into
   * @param nodes The nodes to be added into the graph
   */
  void addNodesToGraph(Graph & graph, std::vector<Json> & nodes);

  /**
   * @brief Add all edges into the graph
   * @param graph The graph in which the edges are added into
   * @param edges The edges to be added into the graph
   */
  void addEdgesToGraph(nav2_route::Graph & graph, std::vector<Json> & edges);
};
}

#endif //NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_INCLUDE_NAV2_ROUTE_PLUGINS_GRAPH_PARSERS_GEOJSON_GRAPH_PARSER_HPP_
