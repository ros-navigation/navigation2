// Copyright (c) 2021
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

#ifndef NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_
#define NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>  

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/string_utils.hpp"

namespace nav2_safety_nodes
{

class SafetyZone : public nav2_util::LifecycleNode
{
public:
    /**
     * @brief A constructor for nav2_safety_nodes::SafetyZone class
     */
    SafetyZone();
    /**
     * @brief A destructor for nav2_safety_nodes::SafetyZone class
     */
    ~SafetyZone() = default;

    /**
     * @brief Configures member variables
     *
     * Initializes action server for "safety_node"
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    
protected:
    /**
     * @brief Action server callbacks
     */
    void timerCallback();

    /**
     * @brief  A callback to handle buffering LaserScan messages
     * @param message The message returned from a message notifier
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr message);
    
    /**
     * @brief  Function to calculate dot product
     * @param vectors Two Eigen vectors of which dot product is to be found
     * @return Dot product of two passed vectors
     */
    double dotProduct(const Eigen::Vector3d & pt1,
        const Eigen::Vector3d & pt2);

    /**
     * @brief  Function for detecting if point is inside or outside of safety zone
     * @param message Pointcloud & safetyzone
     * @return Number of points Inside
     */
    int detectPoints(
      const sensor_msgs::msg::PointCloud2 & cloud,
      const std::vector<geometry_msgs::msg::Point> & safety_zone);

    // The Logger object for logging
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_safety_nodes")};

    std::vector<geometry_msgs::msg::Point> safety_zone_;
    rclcpp::Clock::SharedPtr clock_;

    std::string safety_polygon_;
    double update_frequency_{};
    double zone_action_{};
    int zone_priority_{0};
    int zone_num_pts_{0};
    std::string base_frame_;   ///< The frame_id of the robot base
    double tf_tolerance_{};
    std::vector<std::string> scan_topics_;
    std::string speed_limit_topic_;

    std::shared_ptr<tf2_ros::Buffer> tf2_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    // Used to project laser scans into point clouds
    laser_geometry::LaserProjection projector_;
    std::queue<sensor_msgs::msg::PointCloud2> sensor_data_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      safety_polygon_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr 
      speed_limit_pub_;
    std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>>> 
      scan_subscribers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // end namespace nav2_safety_nodes

#endif  // NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_

