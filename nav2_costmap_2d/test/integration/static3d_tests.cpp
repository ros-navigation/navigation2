//
// Created by sun on 2020/11/3.
//

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "nav2_costmap_2d/nav2_3d_static_layer.hpp"

#include "../testing_helper.hpp"

using namespace std::chrono_literals;

//defined a rclcppfixture
class RclCppFixture
{
public:
    RclCppFixture() {rclcpp::init(0, nullptr);}
    ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
    explicit TestLifecycleNode(const std::string & name)
            : nav2_util::LifecycleNode(name)
    {
    }

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }
};

// create PC2Publisher for publishing 0.5s/pc2 info
class PC2Publisher : public rclcpp::Node
{
    public:
        PC2Publisher(): Node("pc2_publisher")
        {
            _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_info", 10);
            _timer = this->create_wall_timer( 500ms, std::bind(&PC2Publisher::timer_callback, this));
        }

        void CreatePC2Msg()
        {
            sensor_msgs::msg::PointCloud2 test_pc2;
        }
//read from pcd file OR create pc2 msgs?
        void readPCD(sensor_msgs::msg::PointCloud2 pc2)
        {
            pcl::PCLPointCloud2 pclpc2;
            std::string filePath = "/home/sun/Desktop/plugin_github/nav2_3d_costmap_plugin/nav2_3d_costmap_plugin/test/map/diagonalMap.pcd";
            pcl::PCDReader reader;
            reader.read(
                    filePath,
                    pclpc2
                    );
            pcl_conversions::fromPCL(pclpc2, pc2);
        }

    private:
        void timer_callback()
        {
            sensor_msgs::msg::PointCloud2 pc2;
            readPCD(pc2);
            _publisher->publish(pc2);
        }
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

};

class TestNode : public ::testing::Test
{
public:
    TestNode()
    {
        node_ = std::make_shared<TestLifecycleNode>("test_node");
        node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
        node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
        node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
        node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
        node_->declare_parameter(
                "unknown_cost_value",
                rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
        node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
        node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
        node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    }

    ~TestNode() {}

protected:
    std::shared_ptr<TestLifecycleNode> node_;
};

TEST_F(TestNode, testTemplate){
    tf2_ros::Buffer tf(node_->get_clock());

    std::shared_ptr<PC2Publisher> pc2publihser = std::make_shared<PC2Publisher>();

    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

    nav2_costmap_2d::Costmap2D _map_bg;
    _map_bg.setDefaultValue(0);

    std::shared_ptr<nav2_costmap_2d::ObstacleLayer> nlayer = std::make_shared<nav2_costmap_2d::StaticLayer3D>();

    nlayer->initialize(&layers, "static3d", &tf, node_, nullptr, nullptr /*TODO*/);

    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(nlayer));

    addObservation(nlayer, 0.0, 0.0, 1.0 / 2, 0, 0, 1.0 / 2);
    layers.updateMap(0, 0, 0);
    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
    printMap(*costmap);

}




