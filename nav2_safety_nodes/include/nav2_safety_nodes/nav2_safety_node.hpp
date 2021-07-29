#ifndef NAV2_SAFETY_NODES__POLYGON_NODE_HPP_
#define NAV2_SAFETY_NODES__POLYGON_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav2_util/lifecycle_node.hpp"

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

    /**
     * @brief Convert Polygon msg to vector of Points.
     */
    std::vector<geometry_msgs::msg::Point> toPointVector(
      geometry_msgs::msg::Polygon::SharedPtr polygon);

    /**
     * @brief Make the safety_zone from the given string.
     *
     * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
     *
     */
    bool getSafetyZonesFromString(
      const std::string & safety_zone_str,
      std::vector<geometry_msgs::msg::Point> & safety_zone);

    /**
     * @brief Action server callbacks
     */
    void timer_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
    
protected:

    rclcpp::Node::SharedPtr client_node_;


    // Publishers and subscribers
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr safety_polygon_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr safety_polygon_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr safety_polygon_sub_;

    
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    

    /**
     * @brief Get parameters for node
     */
    void getParameters();
    std::string safety_polygon_;
    // float safety_polygon_padding_{0}; 
    double origin_x_{0};
    double origin_y_{0};
    double zone_action_{};
    int zone_priority_{0};
    int zone_num_pts_{0};
    std::string base_frame_;   ///< The frame_id of the robot base

    // Derived parameters
    bool use_polygon_{true};

} 

}
  // end namespace nav2_safety_nodes

#endif  // NAV2_SAFETY_NODES__POLYGON_NODE_HPP_

