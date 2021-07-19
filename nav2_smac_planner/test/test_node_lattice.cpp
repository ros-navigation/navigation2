// Copyright (c) 2021 Joshua Wallace
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
// limitations under the License. Reserved.

#include "nav2_smac_planner/node_lattice.hpp"
#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>

using json = nlohmann::json;

TEST(NodeLatticeTest, parser_test)
{
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
    std::string filePath = pkg_share_dir + "/output.json";
    std::ifstream myJsonFile(filePath);

    ASSERT_TRUE(myJsonFile.is_open());

    json j; 
    myJsonFile >> j;

    nav2_smac_planner::LatticeMetadata metaData; 
    nav2_smac_planner::MotionPrimitive myPrimitive; 
    nav2_smac_planner::MotionPose pose; 
    
    json jsonMetaData = j["latticeMetadata"];
    json jsonPrimatives = j["primitives"];
    json jsonPose = jsonPrimatives[0]["poses"][0];

    nav2_smac_planner::fromJsonToMetaData(jsonMetaData, metaData);

    //Checks for parsing meta data 
    EXPECT_NEAR(metaData.min_turning_radius, 0.4, 0.001);
    EXPECT_NEAR(metaData.primitive_resolution, 0.005, 0.0001);
    EXPECT_NEAR(metaData.grid_resolution, 0.05, 0.001);
    EXPECT_NEAR(metaData.max_length, 1, 0.01);
    EXPECT_NEAR(metaData.number_of_headings, 16, 0.01);
    EXPECT_EQ(metaData.output_file, "output.json");
    EXPECT_NEAR(metaData.heading_angles[0], -180.0, 0.01);

    std::vector<nav2_smac_planner::MotionPrimitive> myPrimitives;
    for(unsigned int i = 0; i < jsonPrimatives.size() ; ++i)
    {
        nav2_smac_planner::MotionPrimitive newPrimative; 
        nav2_smac_planner::fromJsonToMotionPrimitive(jsonPrimatives[i], newPrimative );
        myPrimitives.push_back(newPrimative);
    }

    //Checks for parsing primitives 
    EXPECT_NEAR(myPrimitives[0].trajectory_id, 0, 0.01);
    EXPECT_NEAR(myPrimitives[0].start_angle, 0.0,0.01);
    EXPECT_NEAR(myPrimitives[0].end_angle, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].turning_radius, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].trajectory_length, 0.2, 0.01);
    EXPECT_NEAR(myPrimitives[0].arc_length, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].straight_length, 0.2, 0.01);

    EXPECT_NEAR(myPrimitives[0].poses[0]._x, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[0]._y, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[0]._theta, 0.0, 0.01);

    EXPECT_NEAR(myPrimitives[0].poses[1]._x, 0.06667, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[1]._y, 0.0, 0.01);
    EXPECT_NEAR(myPrimitives[0].poses[1]._theta, 0.0, 0.01);
}

TEST(NodeLatticeTest, test_node_lattice_neighbors)
{
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("nav2_smac_planner");
    std::string filePath = pkg_share_dir + "/output.json";
    
    nav2_smac_planner::SearchInfo info; 
    info.minimum_turning_radius = 1.1; 
    info.non_straight_penalty = 1; 
    info.change_penalty = 1; 
    info.reverse_penalty = 1; 
    info.cost_penalty = 1; 
    info.analytic_expansion_ratio = 1; 
    info.lattice_filepath = filePath;
    info.cache_obstacle_heuristic = true;  

    unsigned int x = 100; 
    unsigned int y = 100;
    unsigned int angle_quantization = 16; 

    nav2_smac_planner::NodeLattice::initMotionModel(nav2_smac_planner::MotionModel::STATE_LATTICE,
                   x,
                   y,
                   angle_quantization,
                   info);

    nav2_smac_planner::NodeLattice aNode(0);
    aNode.setPose( nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 0) );
    nav2_smac_planner::MotionPoses projections = nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives( &aNode);

    EXPECT_NEAR(projections[0]._x, 0.13333, 0.01);
    EXPECT_NEAR(projections[0]._y, 0.0, 0.01);
    EXPECT_NEAR(projections[0]._theta, 0.0, 0.01);

    aNode.setPose(nav2_smac_planner::NodeHybrid::Coordinates(0, 0, 1));

    projections = nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives( &aNode);

    EXPECT_NEAR(projections[0]._x, -0.13333, 0.01);
    EXPECT_NEAR(projections[0]._y, 0.0, 0.01);
    EXPECT_NEAR(projections[0]._theta, 3.14, 0.01);
}