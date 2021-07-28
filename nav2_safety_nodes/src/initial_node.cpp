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

#include <utility>

using namespace std::chrono_literals;

class SafetyZone : public rclcpp::Node
{
  public:
    SafetyZone()
    : Node("safety_zone_node")
    {
    
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(this->get_logger(), "Creating Footprint");
    

    // pass footprint parameters at string
    this->declare_parameter("footprint")   
    rclcpp::Parameter str_param = this->get_parameter("foorprint");

    std::string footprint_ = str_param.as_string();
    

    // footprint subscriber
    footprint_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>("footprint",default_qos,std::bind(&SafetyZone::setRobotFootprintPolygon, this, std::placeholders::_1));

    // footprint publisher
    footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("published_footprint", default_qos);


    // function to convert footprint in vector of points 
     std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
    {
      std::vector<geometry_msgs::msg::Point> pts;
      for (unsigned int i = 0; i < polygon->points.size(); i++) {
        pts.push_back(toPoint(polygon->points[i]));
      }
      return pts;
    }

    use_radius = false;
    // footprint -> point vector and Set the footprint
      if (use_radius_) {
        setRobotFootprint(makeFootprintFromRadius(robot_radius_));
      } else {
        std::vector<geometry_msgs::msg::Point> new_footprint;
        makeFootprintFromString(footprint_, new_footprint);
        toPointVector(new_footprint);
        setRobotFootprint(new_footprint);
      }


    // Laserscan subscriber
    subscriber_= this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", default_qos, std::bind(&SafetyZone::timer_callback, this, std::placeholders::_1));
      
      // Velocity publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer -> 10hz
    timer_ = this->create_wall_timer(
      100ms, std::bind(&SafetyZone::timer_callback, this));
    // }

        

        // makeFootprintFromString function
        bool makeFootprintFromString(
        const std::string & footprint_string,
        std::vector<geometry_msgs::msg::Point> & footprint)
        {
        std::string error;
        std::vector<std::vector<float>> vvf = parseVVF(footprint_string, error);

        if (error != "") {
            RCLCPP_ERROR(
            rclcpp::get_logger(
                "nav2_costmap_2d"), "Error parsing footprint parameter: '%s'", error.c_str());
            RCLCPP_ERROR(
            rclcpp::get_logger(
                "nav2_costmap_2d"), "  Footprint string was '%s'.", footprint_string.c_str());
            return false;
        }

        // convert vvf into points.
        if (vvf.size() < 3) {
            RCLCPP_ERROR(
            rclcpp::get_logger(
                "nav2_costmap_2d"),
            "You must specify at least three points for the robot footprint, reverting to previous footprint."); //NOLINT
            return false;
        }
        footprint.reserve(vvf.size());
        for (unsigned int i = 0; i < vvf.size(); i++) {
            if (vvf[i].size() == 2) {
            geometry_msgs::msg::Point point;
            point.x = vvf[i][0];
            point.y = vvf[i][1];
            point.z = 0;
            footprint.push_back(point);
            } else {
            RCLCPP_ERROR(
                rclcpp::get_logger(
                "nav2_costmap_2d"),
                "Points in the footprint specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
                static_cast<int>(vvf[i].size()));
            return false;
            }
        }

        return true;
        }

        void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
        {
        unpadded_footprint_ = points;
        padded_footprint_ = points;
        padFootprint(padded_footprint_, footprint_padding_);
        layered_costmap_->setFootprint(padded_footprint_);
        }


    }



  private:
    

    void timer_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        
    }


    void setRobotFootprintPolygon(
    const geometry_msgs::msg::Polygon::SharedPtr footprint)
    {
    setRobotFootprint(toPointVector(footprint));
    }


    rclcpp::TimerBase::SharedPtr timer_;

     /// Laser messages subscriber
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;

    ///  Velocity command publisher
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;

     /// Laser messages subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

    ///  Velocity command publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyZone>());   
    rclcpp::shutdown();
    return 0;
  }