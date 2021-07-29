#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_safety_nodes/nav2_safety_node.hpp"

#include <utility>

using namespace std::chrono_literals;

namespace nav2_safety_nodes{

  SafetyZone::SafetyZone()
  : nav2_util::LifecycleNode("SafetyZone", "", true, rclcpp::NodeOptions().arguments())
  {

    RCLCPP_INFO(get_logger(), "Creating Safety Polygon");

    // pass footprint parameters at string
    declare_parameter("safety_polygon_");

  }

  SafetyZone::~SafetyZone()
  {
  }

  nav2_util::CallbackReturn
  SafetyZone::on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    auto node = shared_from_this();
    
    // Getting all parameters
    getParameters();

    // polygon -> point vector
    if(safety_polygon_){
      std::vector<geometry_msgs::msg::Point> new_polygon;
      getSafetyZonesFromString(safety_polygon_, new_polygon);
      toPointVector(new_polygon);
    }

    // Create the publishers and subscribers
    safety_polygon_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
    "safety_polygon_",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&SafetyZone::PolygonCallback, this, std::placeholders::_1));

    safety_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "published_polygon", rclcpp::SystemDefaultsQoS());
    
    // Laserscan subscriber
    subscriber_= create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::SystemDefaultsQoS(), std::bind(&SafetyZone::timer_callback, this, std::placeholders::_1));
    
      // Velocity publisher
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer -> 10hz
    timer_ = create_wall_timer(
      100ms, std::bind(&SafetyZone::timer_callback, this));


      return nav2_util::CallbackReturn::SUCCESS;
    }

  nav2_util::CallbackReturn
  SafetyZone::on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  // Get Parameters
  void
  SafetyZone::getParameters()
  {
    RCLCPP_DEBUG(get_logger(), " getParameters");

    // Get all of the required parameters
    
    get_parameter("safety_polygon", safety_polygon_).as_string();
    get_parameter("footprint_padding", safety_polygon_padding_);
    get_parameter("global_frame", global_frame_);
    get_parameter("height", map_height_meters_);
    get_parameter("origin_x", origin_x_);
    get_parameter("origin_y", origin_y_);
    get_parameter("robot_base_frame", robot_base_frame_);

    auto node = shared_from_this();

    use_polygon_ = true;
    // If the safety_polygon has been specified, it must be in the correct format
    if (safety_polygon_ != "" && safety_polygon_ != "[]") {
      // Polygon parameter has been specified, try to convert it
      std::vector<geometry_msgs::msg::Point> new_polygon;
      if (getSafetyZonesFromString(safety_polygon_, new_polygon)) {
        // The specified safety_polygon is valid
        use_polygon = true;
      } else {
        // Polygon provided but invalid, so stay with the radius
        RCLCPP_ERROR(
          get_logger(), "The safety_polygon is invalid: \"%s\" :) ",
        safety_polygon_.c_str());
      }
    }
  }

  // getSafetyZonesFromString function
  bool SafetyZone::getSafetyZonesFromString(
  const std::string & safety_zone_str,
  std::vector<geometry_msgs::msg::Point> & safety_zone)
  {
      std::string error;
      std::vector<std::vector<float>> vvf = parseVVF(safety_zone_str, error);

      if (error != "") {
          RCLCPP_ERROR(
          rclcpp::get_logger(
              "nav2_safety_nodes"), "Error parsing safety_zone : '%s'", error.c_str());
          RCLCPP_ERROR(
          rclcpp::get_logger(
              "nav2_safety_nodes"), "  Safety_zone string was '%s'.", safety_zone_str.c_str());
          return false;
      }

      // convert vvf into points.
      if (vvf.size() < 3) {
          RCLCPP_ERROR(
          rclcpp::get_logger(
              "nav2_safety_nodes"),
          "You must specify at least three points for the robot safety_zone, reverting to previous safety_zone."); //NOLINT
          return false;
      }
      safety_zone.reserve(vvf.size());
      for (unsigned int i = 0; i < vvf.size(); i++) {
          if (vvf[i].size() == 2) {
          geometry_msgs::msg::Point point;
          point.x = vvf[i][0];
          point.y = vvf[i][1];
          point.z = 0;
          safety_zone.push_back(point);
          } else {
          RCLCPP_ERROR(
              rclcpp::get_logger(
              "nav2_safety_nodes"),
              "Points in the safety_zone specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
              static_cast<int>(vvf[i].size()));
          return false;
          }
      }

      return true;
  }

  // function to convert footprint in vector of points 
  std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
  {
    std::vector<geometry_msgs::msg::Point> pts;
    for (unsigned int i = 0; i < polygon->points.size(); i++) {
      pts.push_back(toPoint(polygon->points[i]));
    }
    return pts;
  }

  void
  SafetyZone::PolygonCallback()
  {
    // empty callback
  }

} // namespace nav2_safety_nodes


  // int main(int argc, char * argv[])
  // {
  //   rclcpp::init(argc, argv);
  //   rclcpp::spin(std::make_shared<SafetyZone>());   
  //   rclcpp::shutdown();
  //   return 0;
  // }