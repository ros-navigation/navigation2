// Copyright (c) 2026 Panav Arpit Raaj
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

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_route/plugins/graph_file_loaders/osm_graph_file_loader.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

// Re-exposes the protected topology helpers so they can be unit tested in
// isolation, with hand-built ways and no file I/O.
class OsmLoaderTestPeer : public OsmGraphFileLoader
{
public:
  using OsmGraphFileLoader::OsmWay;
  using OsmGraphFileLoader::Section;
  using OsmGraphFileLoader::OneWay;
  using OsmGraphFileLoader::doesFileExist;
  using OsmGraphFileLoader::parseOsm;
  using OsmGraphFileLoader::countNodeReferences;
  using OsmGraphFileLoader::splitWaysIntoSections;
  using OsmGraphFileLoader::collectVertexIds;
  using OsmGraphFileLoader::convertCoordinates;
  using OsmGraphFileLoader::addNodesToGraph;
  using OsmGraphFileLoader::parseOneway;
  using OsmGraphFileLoader::addEdgesFromSections;
  using OsmGraphFileLoader::osm_to_nodeid_;
  using OsmGraphFileLoader::next_edge_id_;
};

void writeOsmToFile(const std::string & xml, const std::string & file_path)
{
  std::ofstream out(file_path);
  out << xml;
  out.close();
}

// A non-existent path must fail cleanly (false), never throw.
TEST(OsmGraphFileLoader, test_file_does_not_exist)
{
  Graph graph;
  GraphToIDMap graph_to_id_map;
  std::string file_path;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
}

// Malformed XML must be caught by tinyxml2's LoadFile and reported as failure.
TEST(OsmGraphFileLoader, test_malformed_xml)
{
  const std::string file_path = "malformed.osm";
  writeOsmToFile("<osm><way></osm", file_path);
  Graph graph;
  GraphToIDMap graph_to_id_map;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
  std::filesystem::remove(file_path);
}

// A well-formed file with nodes but no (kept) ways yields no graph -> failure.
TEST(OsmGraphFileLoader, test_no_ways)
{
  const std::string file_path = "no_ways.osm";
  const std::string xml =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<osm version=\"0.6\">\n"
    "  <node id=\"101\" lat=\"40.0\" lon=\"-75.0\"/>\n"
    "  <node id=\"102\" lat=\"40.0009\" lon=\"-75.0\"/>\n"
    "</osm>";
  writeOsmToFile(xml, file_path);
  Graph graph;
  GraphToIDMap graph_to_id_map;
  OsmGraphFileLoader graph_file_loader;
  EXPECT_FALSE(graph_file_loader.loadGraphFromFile(graph, graph_to_id_map, file_path));
  std::filesystem::remove(file_path);
}

// A way whose interior nodes are unshared stays one section; the shape nodes
// ride inside the chain and never become vertices.
TEST(OsmGraphFileLoader, topology_single_way_no_junctions)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ways.push_back({{1, 2, 3}, {{"highway", "track"}}});

  const auto ref_count = peer.countNodeReferences(ways);
  const auto sections = peer.splitWaysIntoSections(ways, ref_count);

  ASSERT_EQ(sections.size(), 1u);
  EXPECT_EQ(sections[0].node_chain, (std::vector<int64_t>{1, 2, 3}));
  EXPECT_EQ(sections[0].tags.at("highway"), "track");
}

// Two ways crossing at a shared middle node: the junction splits both ways,
// and the junction id ends one chain AND begins the next.
TEST(OsmGraphFileLoader, topology_plus_intersection)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ways.push_back({{10, 11, 12, 13, 14}, {{"highway", "track"}}});
  ways.push_back({{20, 12, 21}, {{"highway", "path"}}});

  const auto ref_count = peer.countNodeReferences(ways);
  const auto sections = peer.splitWaysIntoSections(ways, ref_count);

  ASSERT_EQ(sections.size(), 4u);
  EXPECT_EQ(sections[0].node_chain, (std::vector<int64_t>{10, 11, 12}));
  EXPECT_EQ(sections[1].node_chain, (std::vector<int64_t>{12, 13, 14}));
  EXPECT_EQ(sections[2].node_chain, (std::vector<int64_t>{20, 12}));
  EXPECT_EQ(sections[3].node_chain, (std::vector<int64_t>{12, 21}));
  EXPECT_EQ(sections[0].tags.at("highway"), "track");
  EXPECT_EQ(sections[3].tags.at("highway"), "path");
}

// A junction at a way's ENDPOINT must not split that way: endpoints already
// close sections by construction.
TEST(OsmGraphFileLoader, topology_way_through_junction)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ways.push_back({{1, 2, 3}, {{"highway", "track"}}});
  ways.push_back({{9, 3}, {{"highway", "track"}}});

  const auto ref_count = peer.countNodeReferences(ways);
  const auto sections = peer.splitWaysIntoSections(ways, ref_count);

  ASSERT_EQ(sections.size(), 2u);
  EXPECT_EQ(sections[0].node_chain, (std::vector<int64_t>{1, 2, 3}));
  EXPECT_EQ(sections[1].node_chain, (std::vector<int64_t>{9, 3}));
}

// A node revisited within ONE way (figure-8) reaches ref_count 2 and splits
// the way there; the middle section legitimately starts and ends at the same
// node (a loop spur).
TEST(OsmGraphFileLoader, topology_figure_eight)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ways.push_back({{1, 2, 3, 2, 4}, {{"highway", "track"}}});

  const auto ref_count = peer.countNodeReferences(ways);
  const auto sections = peer.splitWaysIntoSections(ways, ref_count);

  ASSERT_EQ(sections.size(), 3u);
  EXPECT_EQ(sections[0].node_chain, (std::vector<int64_t>{1, 2}));
  EXPECT_EQ(sections[1].node_chain, (std::vector<int64_t>{2, 3, 2}));
  EXPECT_EQ(sections[2].node_chain, (std::vector<int64_t>{2, 4}));
}

// The vertex set is exactly the unique section boundaries (junctions +
// endpoints), sorted for deterministic graph indices. Shape nodes (11, 13)
// inside a chain are not vertices.
TEST(OsmGraphFileLoader, collect_vertex_ids_are_sorted_unique_boundaries)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 11, 12}, {}});
  sections.push_back({{12, 13, 14}, {}});
  sections.push_back({{20, 12}, {}});
  sections.push_back({{12, 21}, {}});

  EXPECT_EQ(peer.collectVertexIds(sections), (std::vector<int64_t>{10, 12, 14, 20, 21}));
}

// addNodesToGraph assigns sequential nav2 ids, records the OSM->nav2 mapping,
// and copies coordinates - without ever casting the int64 OSM id into nodeid.
TEST(OsmGraphFileLoader, add_nodes_assigns_sequential_ids_and_coords)
{
  OsmLoaderTestPeer peer;
  const std::vector<int64_t> vertex_ids{10, 12, 14};
  std::unordered_map<int64_t, Coordinates> coords;
  Coordinates a; a.x = 1.0f; a.y = 2.0f; coords[10] = a;
  Coordinates b; b.x = 3.0f; b.y = 4.0f; coords[12] = b;
  Coordinates c; c.x = 5.0f; c.y = 6.0f; coords[14] = c;

  Graph graph;
  GraphToIDMap graph_to_id_map;
  peer.addNodesToGraph(graph, graph_to_id_map, vertex_ids, coords);

  ASSERT_EQ(graph.size(), 3u);
  EXPECT_EQ(graph[0].nodeid, 0u);
  EXPECT_EQ(graph[1].nodeid, 1u);
  EXPECT_NEAR(graph[1].coords.x, 3.0f, 1e-6);
  EXPECT_NEAR(graph[2].coords.y, 6.0f, 1e-6);
  EXPECT_EQ(graph[0].coords.frame_id, "map");
  EXPECT_EQ(graph_to_id_map[0], 0u);
  EXPECT_EQ(peer.osm_to_nodeid_[10], 0u);
  EXPECT_EQ(peer.osm_to_nodeid_[14], 2u);
}

// A junction whose node was missing from the file (no coordinates) is dropped
// rather than producing a coordinate-less vertex.
TEST(OsmGraphFileLoader, add_nodes_drops_vertices_without_coordinates)
{
  OsmLoaderTestPeer peer;
  const std::vector<int64_t> vertex_ids{10, 12, 99};  // 99 = clipped, no coords
  std::unordered_map<int64_t, Coordinates> coords;
  Coordinates z; z.x = 0.0f; z.y = 0.0f;
  coords[10] = z;
  coords[12] = z;

  Graph graph;
  GraphToIDMap graph_to_id_map;
  peer.addNodesToGraph(graph, graph_to_id_map, vertex_ids, coords);

  EXPECT_EQ(graph.size(), 2u);
  EXPECT_EQ(peer.osm_to_nodeid_.count(99), 0u);
}

// Builds a two-vertex graph (ids 10, 14) on the peer so edge wiring can be
// tested without the coordinate-conversion service.
static void buildTwoVertexGraph(
  OsmLoaderTestPeer & peer, Graph & graph, GraphToIDMap & map)
{
  const std::vector<int64_t> vertex_ids{10, 14};
  std::unordered_map<int64_t, Coordinates> coords;
  Coordinates a; a.x = 0.0f; a.y = 0.0f; coords[10] = a;
  Coordinates b; b.x = 1.0f; b.y = 0.0f; coords[14] = b;
  peer.addNodesToGraph(graph, map, vertex_ids, coords);
}

// No oneway tag -> the section becomes two directed edges (both ways).
TEST(OsmGraphFileLoader, edges_bidirectional_without_oneway)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 14}, {{"highway", "track"}}});
  peer.addEdgesFromSections(graph, sections);

  ASSERT_EQ(graph.size(), 2u);
  EXPECT_EQ(graph[0].neighbors.size(), 1u);  // 10 -> 14
  EXPECT_EQ(graph[1].neighbors.size(), 1u);  // 14 -> 10
}

// oneway=yes -> a single edge along the node order only.
TEST(OsmGraphFileLoader, edges_oneway_yes_is_forward_only)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 14}, {{"highway", "track"}, {"oneway", "yes"}}});
  peer.addEdgesFromSections(graph, sections);

  EXPECT_EQ(graph[0].neighbors.size(), 1u);  // 10 -> 14
  EXPECT_EQ(graph[1].neighbors.size(), 0u);
}

// oneway=-1 -> a single edge against the node order.
TEST(OsmGraphFileLoader, edges_oneway_reverse_is_backward_only)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 14}, {{"highway", "track"}, {"oneway", "-1"}}});
  peer.addEdgesFromSections(graph, sections);

  EXPECT_EQ(graph[0].neighbors.size(), 0u);
  EXPECT_EQ(graph[1].neighbors.size(), 1u);  // 14 -> 10
}

// A section whose boundary never became a vertex (clipped extract) is skipped,
// not connected to a phantom node.
TEST(OsmGraphFileLoader, edges_skip_section_with_unresolved_boundary)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 99}, {{"highway", "track"}}});  // 99 is not a vertex
  peer.addEdgesFromSections(graph, sections);

  EXPECT_EQ(graph[0].neighbors.size(), 0u);
  EXPECT_EQ(graph[1].neighbors.size(), 0u);
}

// End-to-end on the packaged sample .osm (no service needed): parse + topology
// must yield the documented structure. Guards against regressions on a real,
// file-shaped input rather than only hand-built structs.
TEST(OsmGraphFileLoader, sample_graph_resolves_to_expected_topology)
{
  OsmLoaderTestPeer peer;
  const std::string path =
    nav2::get_package_share_directory("nav2_route") + "/graphs/sample_graph.osm";

  std::unordered_map<int64_t, std::pair<double, double>> nodes;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ASSERT_TRUE(peer.parseOsm(path, nodes, ways));
  EXPECT_EQ(nodes.size(), 7u);
  EXPECT_EQ(ways.size(), 2u);  // both ways are kept

  const auto ref = peer.countNodeReferences(ways);
  const auto sections = peer.splitWaysIntoSections(ways, ref);
  ASSERT_EQ(sections.size(), 4u);  // each way splits at the shared node 1003

  const auto vertices = peer.collectVertexIds(sections);
  EXPECT_EQ(vertices, (std::vector<int64_t>{1001, 1003, 1005, 1006, 1007}));
}

// parseOneway covers every recognized value, plus unknown/absent -> bidirectional.
TEST(OsmGraphFileLoader, parse_oneway_recognizes_all_values)
{
  OsmLoaderTestPeer peer;
  using OneWay = OsmLoaderTestPeer::OneWay;
  EXPECT_EQ(peer.parseOneway({{"oneway", "yes"}}), OneWay::FORWARD);
  EXPECT_EQ(peer.parseOneway({{"oneway", "true"}}), OneWay::FORWARD);
  EXPECT_EQ(peer.parseOneway({{"oneway", "1"}}), OneWay::FORWARD);
  EXPECT_EQ(peer.parseOneway({{"oneway", "-1"}}), OneWay::REVERSE);
  EXPECT_EQ(peer.parseOneway({{"oneway", "reverse"}}), OneWay::REVERSE);
  EXPECT_EQ(peer.parseOneway({{"oneway", "no"}}), OneWay::BOTH);
  EXPECT_EQ(peer.parseOneway({{"oneway", "false"}}), OneWay::BOTH);
  EXPECT_EQ(peer.parseOneway({{"oneway", "0"}}), OneWay::BOTH);
  EXPECT_EQ(peer.parseOneway({{"oneway", "backwards"}}), OneWay::BOTH);  // unknown -> permissive
  EXPECT_EQ(peer.parseOneway({}), OneWay::BOTH);                          // absent -> bidirectional
}

// The loader applies no filtering, tags are preserved for scoring
TEST(OsmGraphFileLoader, parse_keeps_way_without_highway_tag)
{
  const std::string file_path = "no_highway_way.osm";
  writeOsmToFile(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<osm version=\"0.6\">\n"
    "  <node id=\"1\" lat=\"40.0\" lon=\"-75.0\"/>\n"
    "  <node id=\"2\" lat=\"40.001\" lon=\"-75.0\"/>\n"
    "  <way id=\"5\">\n"
    "    <nd ref=\"1\"/>\n"
    "    <nd ref=\"2\"/>\n"
    "    <tag k=\"barrier\" v=\"gate\"/>\n"  // deliberately not a highway tag
    "  </way>\n"
    "</osm>", file_path);
  OsmLoaderTestPeer peer;
  std::unordered_map<int64_t, std::pair<double, double>> nodes;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ASSERT_TRUE(peer.parseOsm(file_path, nodes, ways));
  ASSERT_EQ(ways.size(), 1u);                     // kept despite no highway tag
  EXPECT_EQ(ways[0].tags.at("barrier"), "gate");  // tags preserved for scoring
  std::filesystem::remove(file_path);
}

// A way with fewer than two refs has no extent and yields no section.
TEST(OsmGraphFileLoader, single_ref_way_produces_no_section)
{
  OsmLoaderTestPeer peer;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ways.push_back({{42}, {{"highway", "track"}}});
  const auto ref = peer.countNodeReferences(ways);
  EXPECT_TRUE(peer.splitWaysIntoSections(ways, ref).empty());
}

// A <node> missing a coordinate is skipped during parsing, not stored as (0,0).
TEST(OsmGraphFileLoader, parse_skips_node_missing_coordinate)
{
  const std::string file_path = "missing_coord.osm";
  writeOsmToFile(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<osm version=\"0.6\">\n"
    "  <node id=\"1\" lat=\"40.0\" lon=\"-75.0\"/>\n"
    "  <node id=\"2\" lat=\"40.0\"/>\n"  // missing lon -> skipped
    "</osm>", file_path);
  OsmLoaderTestPeer peer;
  std::unordered_map<int64_t, std::pair<double, double>> nodes;
  std::vector<OsmLoaderTestPeer::OsmWay> ways;
  ASSERT_TRUE(peer.parseOsm(file_path, nodes, ways));
  EXPECT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.count(2), 0u);
  std::filesystem::remove(file_path);
}

// A self-loop section (both boundaries the same junction) adds no edge.
TEST(OsmGraphFileLoader, edges_skip_self_loop_section)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);  // vertices 10, 14

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 11, 10}, {{"highway", "track"}}});  // closes back on 10
  peer.addEdgesFromSections(graph, sections);

  EXPECT_EQ(graph[0].neighbors.size(), 0u);
}

// Edges point at the correct destination node and get sequential ids.
TEST(OsmGraphFileLoader, edges_point_to_correct_nodes_and_assign_sequential_ids)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);  // 10 -> idx 0, 14 -> idx 1

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 14}, {{"highway", "track"}}});  // bidirectional
  peer.addEdgesFromSections(graph, sections);

  ASSERT_EQ(graph[0].neighbors.size(), 1u);
  ASSERT_EQ(graph[1].neighbors.size(), 1u);
  EXPECT_EQ(graph[0].neighbors[0].end, &graph[1]);
  EXPECT_EQ(graph[1].neighbors[0].end, &graph[0]);
  EXPECT_EQ(peer.next_edge_id_, 2u);  // two directed edges -> ids 0 and 1
}

// Two ways between the same junctions produce two parallel edges (no dedup in v1).
TEST(OsmGraphFileLoader, parallel_ways_produce_parallel_edges)
{
  OsmLoaderTestPeer peer;
  Graph graph;
  GraphToIDMap map;
  buildTwoVertexGraph(peer, graph, map);

  std::vector<OsmLoaderTestPeer::Section> sections;
  sections.push_back({{10, 14}, {{"highway", "track"}, {"oneway", "yes"}}});
  sections.push_back({{10, 14}, {{"highway", "path"}, {"oneway", "yes"}}});
  peer.addEdgesFromSections(graph, sections);

  EXPECT_EQ(graph[0].neighbors.size(), 2u);
}
