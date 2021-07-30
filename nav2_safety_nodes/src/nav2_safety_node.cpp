#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
#include <sstream>
#include <algorithm>
#include <limits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_safety_nodes/nav2_safety_node.hpp"



using namespace std::chrono_literals;

namespace nav2_safety_nodes{

  SafetyZone::SafetyZone()
  : nav2_util::LifecycleNode("SafetyZone", "", true, rclcpp::NodeOptions().arguments())
  {
    RCLCPP_INFO(logger_, "Creating Safety Polygon");

    // pass polygon parameters at string
    declare_parameter("safety_polygon", rclcpp::ParameterValue(std::string("[]")));
    declare_parameter("zone_action", rclcpp::ParameterValue(0.0));
    declare_parameter("zone_priority", rclcpp::ParameterValue(1));
    declare_parameter("zone_num_pts", rclcpp::ParameterValue(1));
    declare_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
  }

  SafetyZone::~SafetyZone()
  {
  }

  nav2_util::CallbackReturn
  SafetyZone::on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(logger_, "Configuring");

    auto node = shared_from_this();
    // Getting all parameters
    getParameters();
      return nav2_util::CallbackReturn::SUCCESS;
    }

  nav2_util::CallbackReturn
  SafetyZone::on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(logger_, "Activating");
    // Create the publishers and subscribers
    safety_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "published_polygon", rclcpp::SystemDefaultsQoS());
    // Laserscan subscriber
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan",
                  rclcpp::SystemDefaultsQoS(),
                  std::bind(&SafetyZone::timer_callback, this, std::placeholders::_1));
      // Velocity publisher
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer -> 10hz
    timer_ = create_wall_timer(
      100ms, std::bind(&SafetyZone::timer_callback, this));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(logger_, "Deactivating");
    safety_polygon_pub_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(logger_, "Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  SafetyZone::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(logger_, "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  // Get Parameters
  void
  SafetyZone::getParameters()
  {
    RCLCPP_DEBUG(logger_, " getParameters");

    // Get all of the required parameters
    get_parameter("safety_polygon", safety_polygon_).as_string();
    getparameter("zone_action", zone_action_)
    getparameter("zone_priority", zone_priority_)
    getparameter("zone_num_pts", zone_num_pts_)
    get_parameter("base_frame", base_frame_);

    auto node = shared_from_this();

    // If the safety_polygon has been specified, it must be in the correct format
    if (safety_polygon_ != "" && safety_polygon_ != "[]") {
      // Polygon parameter has been specified, polygon -> point vector
      std::vector<geometry_msgs::msg::Point> new_polygon;
      // parsing polygon parameters
      if(getSafetyZonesFromString(safety_polygon_, new_polygon)){
        // converting into point vector
        toPointVector(new_polygon);
      }else {
        // Polygon provided but invalid, so stay with the radius
        RCLCPP_ERROR(
        logger_, "The safety_polygon is invalid: \"%s\" :) ",
        safety_polygon_.c_str());
        }
      if(safety_polygon_){
        std::vector<geometry_msgs::msg::Point> new_polygon;
        // parsing polygon parameters
        getSafetyZonesFromString(safety_polygon_, new_polygon);
        // converting into point vector
        toPointVector(new_polygon);
      }
    }
  }

  // parsing polygon parameters
  bool SafetyZone::getSafetyZonesFromString(
  const std::string & safety_zone_str,
  std::vector<geometry_msgs::msg::Point> & safety_zone)
  {
      std::string error;
      std::vector<std::vector<float>> vvf = parseVVF(safety_zone_str, error);

      if (error != "") {
          RCLCPP_ERROR(
          logger_, "Error parsing safety_zone : '%s'", error.c_str());
          RCLCPP_ERROR(
          logger_, "  Safety_zone string was '%s'.", safety_zone_str.c_str());
          return false;
      }

      // convert vvf into points.
      if (vvf.size() < 3) {
          RCLCPP_ERROR(
          logger_,
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
              logger_,
              "Points in the safety_zone specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
              static_cast<int>(vvf[i].size()));
          return false;
          }
      }

      return true;
  }
  // Parse a vector of vector of floats from a string.
  std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
  {
    std::vector<std::vector<float>> result;

    std::stringstream input_ss(input);
    int depth = 0;
    std::vector<float> current_vector;
    while (!!input_ss && !input_ss.eof()) {
      switch (input_ss.peek()) {
        case EOF:
          break;
        case '[':
          depth++;
          if (depth > 2) {
            error_return = "Array depth greater than 2";
            return result;
          }
          input_ss.get();
          current_vector.clear();
          break;
        case ']':
          depth--;
          if (depth < 0) {
            error_return = "More close ] than open [";
            return result;
          }
          input_ss.get();
          if (depth == 1) {
            result.push_back(current_vector);
          }
          break;
        case ',':
        case ' ':
        case '\t':
          input_ss.get();
          break;
        default:  // All other characters should be part of the numbers.
          if (depth != 2) {
            std::stringstream err_ss;
            err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
            error_return = err_ss.str();
            return result;
          }
          float value;
          input_ss >> value;
          if (!!input_ss) {
            current_vector.push_back(value);
          }
          break;
      }
    }

    if (depth != 0) {
      error_return = "Unterminated vector string.";
    } else {
      error_return = "";
    }

    return result;
  }
  // function to convert footprint in vector of points
  std::vector<geometry_msgs::msg::Point>
      toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
  {
    std::vector<geometry_msgs::msg::Point> pts;
    for (unsigned int i = 0; i < polygon->points.size(); i++) {
      pts.push_back(toPoint(polygon->points[i]));
    }
    return pts;
  }


}  // namespace nav2_safety_nodes
