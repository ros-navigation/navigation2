#ifndef NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_
#define NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
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
    ~SafetyZone();

    /**
     * @brief Configures member variables
     *
     * Initializes action server for "follow_waypoints"
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

    /**
       * @brief Get the pose of the robot in the global frame of the costmap
       * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
       * @return True if the pose was set successfully, false otherwise
       */
    bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);
    
    // /**
    //  * @brief Make the safety_zone from the given string.
    //  *
    //  * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
    //  *
    //  */
    // bool makeVectorPointsFromString(
    //   const std::string & safety_polygon_, 
    //   std::vector<geometry_msgs::msg::Point> & safety_zone);

    /**
     * @brief Action server callbacks
     */
    void timer_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);

    /**
     * @brief  A callback to handle buffering LaserScan messages
     * @param message The message returned from a message notifier
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);

    // The Logger object for logging
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_safety_nodes")};
  
protected:
    rclcpp::Node::SharedPtr client_node_;
    // Publishers and subscribers
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
                      safety_polygon_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    /**
     * @brief Get parameters for node
     */
    void getParameters();
    std::string safety_polygon_;
    double zone_action_{};
    int zone_priority_{0};
    int zone_num_pts_{0};
    std::string base_frame_;   ///< The frame_id of the robot base
    
    // derived parameters 
    tf2::Duration tf_tolerance_;
    tf2_ros::Buffer & tf2_buffer_;

  /// @brief Used to project laser scans into point clouds
    laser_geometry::LaserProjection projector_;
}

}  // end namespace nav2_safety_nodes

#endif  // NAV2_SAFETY_NODES__NAV2_SAFETY_NODE_HPP_

