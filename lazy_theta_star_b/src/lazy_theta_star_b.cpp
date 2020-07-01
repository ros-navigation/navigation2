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
#include "lazy_theta_star_b/lazy_theta_star_b.h"
#include "nav2_util/node_utils.hpp"
#include "chrono"

namespace lazy_theta_star_b {
    void LazyThetaStarB::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
        node_ = parent;
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
                node_, name_ + ".how_many_corners", rclcpp::ParameterValue(
                        8));

        node_->get_parameter(name_ + ".how_many_corners", how_many_corners_);

        if (how_many_corners_ != 8)
            how_many_corners_ = 4;

        nav2_util::declare_parameter_if_not_declared(
                node_, name_ + ".interpolation_dist", rclcpp::ParameterValue(
                        0.1));

        node_->get_parameter(name_ + ".interpolation_dist", interpolation_dist);

    }

    void LazyThetaStarB::cleanup() {
        RCLCPP_INFO(
                node_->get_logger(), "CleaningUp plugin %s of type LazyThetaStarB",
                name_.c_str());
    }

    void LazyThetaStarB::activate() {
        RCLCPP_INFO(
                node_->get_logger(), "Activating plugin %s of type LazyThetaStarB",
                name_.c_str());
    }

    void LazyThetaStarB::deactivate() {
        RCLCPP_INFO(
                node_->get_logger(), "Deactivating plugin %s of type LazyThetaStarB",
                name_.c_str());
    }

    nav_msgs::msg::Path LazyThetaStarB::createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) {


        RCLCPP_INFO(
                node_->get_logger(), "Path Search Begins");

        costmap_->worldToMap(start.pose.position.x, start.pose.position.y, reinterpret_cast<unsigned int &>(src.x),
                             reinterpret_cast<unsigned int &>(src.y));


        costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, reinterpret_cast<unsigned int &>(dst.x),
                             reinterpret_cast<unsigned int &>(dst.y));
        src.x = int(src.x);
        src.y = int(src.y);

        dst.x = int(dst.x);
        dst.y = int(dst.y);

        data.clear();
        pq.clear();
        path.clear();
        posn.clear();
        lethal_cost = LETHAL_COST;

        sizeX = costmap_->getSizeInCellsX();
        sizeY = costmap_->getSizeInCellsY();

        //has been added so that if the robot is stuck at a place with costmap cost higher than 127, then only for that run, the planner be able to give a path
        if (costmap_->getCost(src.x, src.y) >= lethal_cost && costmap_->getCost(src.x, src.y) <= 253) {
            lethal_cost = costmap_->getCost(src.x, src.y);
        }
        
        initializePosn();
        data.reserve(int((sizeX) * (sizeY) * 0.01));

        clearRobotCell(src.x, src.y);

        RCLCPP_INFO(
                node_->get_logger(), "GOT THE SOURCE AND DESTINATION ------ %i, %i && %i, %i",
                src.x, src.y, dst.x, dst.y);


        if (!isSafe(src.x, src.y) || !isSafe(dst.x, dst.y) || !withinLimits(src.x, src.y) ||
            !withinLimits(dst.x, dst.y)) {
            RCLCPP_INFO(node_->get_logger(), "NO PATH POSSIBLE!!!!!!");
            global_path.poses.clear();
            return global_path;

        }

        data.push_back({(src.x), (src.y), 0, dist(src.x, src.y, dst.x, dst.y), 0, -1,
                        dist(src.x, src.y, dst.x, dst.y)});

        addIndex(src.x, src.y, 0);

        pq.push_back({0, &(data[0].g), &(data[0].f)});
        //added as a buffer, since my binary_heap_del_min starts from the index 1 instead of 0
        pq.push_back({0, &(data[0].g), &(data[0].f)});


        //The algorithm begins here
        id curr_id = 0;
        id id_gen = 1;
        id last_id = 0;
        int cy;
        int cx;
        cx = (data[curr_id].x);
        cy = (data[curr_id].y);

        if (!los_check(src.x, src.y, dst.x, dst.y)) {
            
            while (pq.size() > 1) {

                data[curr_id].counter++;
               
                //          The condition if current point is the destination
                if (cx == dst.x && cy == dst.y) {
                    if (los_check(cx, cy, data[data[last_id].parentId].x, data[data[last_id].parentId].y)) {
                        data[curr_id].parentId = data[last_id].parentId;
                        data[curr_id].g = data[data[last_id].parentId].g;
                        data[curr_id].f = data[data[last_id].parentId].f;
                        break;
                    } else {
                        data[curr_id].parentId = last_id;
                        data[curr_id].g = data[last_id].g;
                        data[curr_id].f = data[last_id].f;
                        break;
                    }
                }

                //generates the child nodes location
                int moves[8][2] = {{cx,     cy + 1},
                                   {cx + 1, cy},
                                   {cx,     cy - 1},
                                   {cx - 1, cy},
                                   {cx + 1, cy + 1},
                                   {cx + 1, cy - 1},
                                   {cx - 1, cy - 1},
                                   {cx - 1, cy + 1}};

                if (how_many_corners != 8) {
                    how_many_corners = 4;
                }

                int mx, my;
                id m_id;
                id curr_par = (data[curr_id].parentId);
                
                //lazy checking for the parent
                if (!(los_check(cx, cy, data[curr_par].x, data[curr_par].y)) && isSafe(cx, cy) &&
                    isSafe(data[curr_par].x, data[curr_par].y)) {
                    
                    double min_dist = INF_COST;
                    int min_dist_id = curr_par;

                    for (int i = 0; i < how_many_corners; i++) {
                        mx = moves[i][0];
                        my = moves[i][1];
                        if (withinLimits(mx, my)) {
                            m_id = getIndex(mx, my);
                            if (m_id != 0) {
                                if (data[m_id].f < min_dist) {
                                    min_dist = data[m_id].f;
                                    min_dist_id = m_id;
                                }
                            }
                        }
                        data[curr_id].parentId = min_dist_id;
                        data[curr_id].g = data[min_dist_id].g + dist(cx, cy, data[min_dist_id].x, data[min_dist_id].y);
                        data[curr_id].f = data[curr_id].g + data[curr_id].h;
                    }
                }
                curr_par = data[curr_id].parentId;

                for (int i = 0; i < how_many_corners; i++) {
                    mx = moves[i][0];
                    my = moves[i][1];

                    if (mx == data[curr_par].x && my == data[curr_par].y)
                        continue;

                    if (withinLimits(mx, my)) {
                        double g_cost = data[curr_par].g + dist(mx, my, data[curr_par].x, data[curr_par].y);
                        double h_cost, cal_cost;
                        m_id = getIndex(mx, my);
                        if (m_id == 0 && isSafe(mx, my)) {
                            h_cost = dist(mx, my, dst.x, dst.y);
                            cal_cost = g_cost + h_cost;
                            data.push_back({int(mx), int(my), g_cost, h_cost, curr_par, -1, cal_cost});
                            addIndex(mx, my, id_gen);
                            push_to_pq(id_gen);
                            id_gen++;
                            continue;
                        } else if (m_id != 0) {
                            h_cost = data[m_id].h;
                            cal_cost = g_cost + h_cost;
                            if (data[m_id].f > cal_cost) {
                                data[m_id].g = g_cost;
                                data[m_id].f = cal_cost;
                                data[m_id].parentId = curr_par;
                                if (data[m_id].closed == 1) {
                                    data[m_id].closed = -1;
                                    push_to_pq(m_id);
                                }
                            }
                            continue;
                        }
                    }
                }

                last_id = curr_id;
                do {
                    curr_id = pq[1].pos_id;
                    cx = data[curr_id].x;
                    cy = data[curr_id].y;
                } while (!isSafe(cx, cy));
                binary_heap_del_min();
                data[curr_id].closed = -1;

            }

        } else {
            RCLCPP_INFO(node_->get_logger(), "Straight Path");
            data.push_back({(dst.x), (dst.y), dist(cx, cy, src.x, src.y), 0, 0});
            curr_id = 1;
        }

        RCLCPP_INFO(node_->get_logger(), "REACHED DEST  %i,   %i", dst.x, dst.y);

        backtrace(&path, curr_id);

        cout << data.size() << '\n';

        cout << "Publishing the Path" << '\n';

        //reset all the values
        global_path = linearInterpolation(path, interpolation_dist);

        geometry_msgs::msg::PoseStamped p_end;
        p_end.pose.position.x = goal.pose.position.x;
        p_end.pose.position.y = goal.pose.position.y;
        p_end.header.stamp = node_->now();
        p_end.header.frame_id = global_frame_;
        global_path.poses.push_back(p_end);

        return global_path;
    }

    void LazyThetaStarB::initializePosn() {
        if (pq.size() != abs(sizeX * sizeY)) {
            for (int i = 0; i < sizeY * sizeX; i++) {
                posn.push_back(0);
            }
        } else {
            for (int i = 0; i < sizeY * sizeX; i++) {
                posn[i] = 0;
            }
        }
    }

    void LazyThetaStarB::addIndex(int cx, int cy, id index) {
        posn[sizeX * cy + cx] = index;
    }

    id LazyThetaStarB::getIndex(int cx, int cy) {
        return posn[sizeX * cy + cx];
    }

    void LazyThetaStarB::backtrace(vector<id> *waypt, id curr_id) {
        vector<id> waypt_rev;
        do {
            waypt_rev.push_back(curr_id);
            curr_id = data[curr_id].parentId;
        } while (curr_id != 0);
        waypt_rev.push_back(0);

        for (int i = waypt_rev.size() - 1; i >= 0; i--) {
            waypt->push_back(waypt_rev[i]);
        }

    }

    void LazyThetaStarB::binary_heap_del_min() {
        int hole = 1;
        int succ = 2, size = pq.size() - 1, sz = size;
        //the main bottom up part where it compares and assigns the smallest value to the hole
        //remember that for this part the heap starts at index 1
        while (succ < sz) {
            double k1 = *(pq[succ].f);
            double k2 = *(pq[succ + 1].f);
            if (k1 > k2) {
                succ++;
                pq[hole] = pq[succ];
            } else {
                pq[hole] = pq[succ];
            }
            hole = succ;
            succ <<= 1;
        }
        //this part checks if the value to be used to fill the last row's hole is small or not
        //if not small then it slides the small value up till it reaches the apt position
        double bubble = *(pq[sz].f);
        int pred = hole >> 1;
        while (*(pq[pred].f) > bubble) {
            pq[hole] = pq[pred];
            hole = pred;
            pred >>= 1;
        }
        //this part simply assigns the last value in the heap to the hole after checking it
        //for maintaining the heap data structure
        pq[hole] = pq[sz];
        pq.pop_back();
    }

    bool LazyThetaStarB::los_check(int x0, int y0, int x1, int y1) {

        int dy = y1 - y0, dx = x1 - x0, f = 0;
        int sx, sy;

        if (dy < 0) {
            dy = -dy;
            sy = -1;
        } else {
            sy = 1;
        }

        if (dx < 0) {
            dx = -dx;
            sx = -1;
        } else {
            sx = 1;
        }

        if (dx >= dy) {
            while (x0 != x1) {
                f += dy;
                if (f >= dx) {
                    if (!isSafe(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2))
                        return false;
                    y0 += sy;
                    f -= dx;
                }
                if (f != 0 && !isSafe(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2))
                    return false;
                if (dy == 0 && !isSafe(x0 + (sx - 1) / 2, y0) && !isSafe(x0 + (sx - 1) / 2, y0 - 1))
                    return false;
                x0 += sx;
            }
        } else {
            while (y0 != y1) {
                f = f + dx;
                if (f >= dy) {
                    if (!isSafe(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2))
                        return false;
                    x0 += sx;
                    f -= dy;
                }
                if (f != 0 && !isSafe(x0 + (sx - 1) / 2, y0 + (sy - 1) / 2))
                    return false;
                if (dx == 0 && !isSafe(x0, y0 + (sy - 1) / 2) && !isSafe(x0 - 1, y0 + (sy - 1) / 2))
                    return false;
                y0 += sy;
            }
        }

        return true;

    }

    void LazyThetaStarB::push_to_pq(id id_this) {
        pq.push_back({id_this, &(data[id_this].g), &(data[id_this].f)});
        push_heap(pq.begin() + 1, pq.end(), comp);
    }

    bool LazyThetaStarB::isSafe(int cx, int cy) {
        return (costmap_->getCost(cx, cy) < lethal_cost);
    }

    bool LazyThetaStarB::withinLimits(int x, int y) {
        return (x > 0 && x <= sizeX && y > 0 && y <= sizeY);
    }

    pts<double> LazyThetaStarB::tf_map_to_world(int x, int y) {
        pts<double> pt{};
        pt.x = double(
                costmap_->getOriginX() + (x + 0.5) * costmap_->getResolution());
        pt.y = double(
                costmap_->getOriginY() + (y + 0.5) * costmap_->getResolution());
        // 0.5 added to display the points at the centre of the grid cell
        return pt;
    }

    nav_msgs::msg::Path LazyThetaStarB::linearInterpolation(vector<id> lp, double dist_bw_points) {

        nav_msgs::msg::Path pa;
        pa.header.frame_id = "map";
        for (int i = 0; i < int(lp.size() - 1); i++) {
            geometry_msgs::msg::PoseStamped p;
            pts<double> pt_copy = tf_map_to_world(data[lp[i]].x, data[lp[i]].y);


            p.pose.position.x = pt_copy.x;
            p.pose.position.y = pt_copy.y;
            p.header.stamp = node_->now();
            p.header.frame_id = global_frame_;
            pa.poses.push_back(p);

            pts<double> pt2_copy = tf_map_to_world(data[lp[i + 1]].x, data[lp[i + 1]].y);

            double distance = double(dist(pt2_copy.x, pt2_copy.y, pt_copy.x, pt_copy.y));
            int loops = int(distance / dist_bw_points);
            double sin_alpha = (pt2_copy.y - pt_copy.y) / distance;
            double cos_alpha = (pt2_copy.x - pt_copy.x) / distance;
            for (int j = 1; j < loops; j++) {
                geometry_msgs::msg::PoseStamped p1;
                p1.pose.position.x = pt_copy.x + j * dist_bw_points * cos_alpha;
                p1.pose.position.y = pt_copy.y + j * dist_bw_points * sin_alpha;
                p1.header.stamp = node_->now();
                p1.header.frame_id = global_frame_;
                pa.poses.push_back(p1);
            }
            geometry_msgs::msg::PoseStamped p2;
            p2.pose.position.x = pt2_copy.x;
            p2.pose.position.y = pt2_copy.y;
            p2.header.stamp = node_->now();
            p2.header.frame_id = global_frame_;
            pa.poses.push_back(p2);

        }


        return pa;

    }


    void LazyThetaStarB::clearRobotCell(int mx, int my) {
        costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }


    //Code to preferably not to look at

    //this finds the shortest distance between the points of the raw path given by the planner
//    nav_msgs::msg::Path LazyThetaStarB::AutoLinearInterpolation(vector<id> lp) {
//
//        nav_msgs::msg::Path the_path;
//        if (lp.size() > 2) {
//            double min_dist = INF_COST;
//
//            for (int i = 0; i < int(lp.size() - 1); i++) {
//                pts<double> pt_copy = tf_map_to_world(data[lp[i]].x, data[lp[i]].y);
//                pts<double> pt2_copy = tf_map_to_world(data[lp[i + 1]].x, data[lp[i + 1]].y)
//                double dist_curr = dist(pt_copy.x, pt_copy.y, pt2_copy.x, pt2_copy.y);
//
//                if (dist_curr < min_dist)
//                    min_dist = dist_curr;
//            }
//
//            the_path = linearInterpolation(lp, min_dist);
//
//        } else {
//            the_path = linearInterpolation(lp, 0.01);
//        }
//        RCLCPP_INFO(node_->get_logger(), "SENT BACK THE PATH!!!!!!");
//        return the_path;
//    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lazy_theta_star_b::LazyThetaStarB, nav2_core::GlobalPlanner)
