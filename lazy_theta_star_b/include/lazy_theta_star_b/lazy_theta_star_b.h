/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Anshumaan Singh
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Anshumaan Singh
 *********************************************************************/
#include<cmath>
#include<iostream>
#include<cstdlib>
#include<cstring>
#include <string>
#include<queue>
#include<algorithm>

#include "rclcpp/rclcpp.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace lazy_theta_star_b {
    typedef int id;
    using namespace std;

#define SMALL 0.001
#define INF_COST 10000000.0
#define LETHAL_COST 100

    template<typename ptsT>
    struct pts {
        ptsT x, y;
    };

    struct pos {
        id pos_id;
        double *g;
        double *f;
    };

    struct tree_node {
        int x, y;
        double g = INF_COST;
        double h = INF_COST;
        id parentId;
        int closed = 0;       //0 - unexplored, 1 - closed, -1 - open
        double f = INF_COST;
        int counter = 0;
        int got_here = 0;
    };


    class LazyThetaStarB : public nav2_core::GlobalPlanner {

    public:
        
        int lethal_cost = LETHAL_COST;

        std::shared_ptr <tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;

        double interpolation_dist;
        int how_many_corners_;
        pts<int> src_{}, dst_{};
        
        //stores the cell data
        vector<tree_node> data;
        
        //stores the raw path given by the planner on the basis of their index in the vector data
        vector<id> path;
        nav_msgs::msg::Path global_path;
        
        // is the priority queue
        vector<pos> pq;
        int sizeX = 0;
        int sizeY = 0;
        int how_many_corners;
        pts<int> src{}, dst{};


        void configure(
                rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                std::string name, std::shared_ptr <tf2_ros::Buffer> tf,
                std::shared_ptr <nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;

        void activate() override;

        void deactivate() override;

        nav_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PoseStamped &start,
                const geometry_msgs::msg::PoseStamped &goal) override;


        //The Line of Sight checking algorithm, takes in the points, and follows the line joining points
        bool los_check(int x0, int y0, int x1, int y1);

        //Gives out the Euclidean Distance
        double dist(double ax, double ay, double bx, double by) {
            return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
        }

        //to pop the minimum value of the queue, it is ever so slightly faster than the stl one, 
        //based on the implementation by Peter Sanders (2000)
        void binary_heap_del_min();

        //to compare between values in thet priority queue
        static bool comp(pos p1, pos p2) {
            return (*(p1.f) != *(p2.f)) ? (*(p1.f) > *(p2.f)) : (*(p1.g) > *(p2.g));
        }

        nav_msgs::msg::Path linearInterpolation(vector<id> lp, double dist_bw_points);

        pts<double> tf_map_to_world(int x, int y) ;
        
        bool withinLimits(int x, int y);

        bool isSafe(int cx, int cy);

        void push_to_pq(id id_this);

        void clearRobotCell(int mx, int my);

        void backtrace(vector <id> *waypt, id curr_id);

        //stores the index at which the node data is stored for a particular co-ordinate
        vector<id> posn;

        //it sets/resets all the values within posn to 0
        void initializePosn();

        //functions used to maintain indices
        void addIndex(int cx, int cy, id index);
        id getIndex(int cx, int cy);

        //nav_msgs::msg::Path AutoLinearInterpolation(vector<id> lp);
    };

}

