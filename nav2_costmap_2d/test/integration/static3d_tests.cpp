#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gtest/gtest.h"
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

class PC2Publisher : public rclcpp::Node
{
public:
    PC2Publisher(): Node("pc2_publisher")
    {
        CreatePC2Msg();
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);
        _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pc2_info", map_qos);
        _publisher->publish(_pc2);
    }
    ~PC2Publisher(){
        _publisher.reset();
    }

    void CreatePC2Msg()
    {
        int height = 100;
        int width = 100;
        _pc2.set__height(1);
        _pc2.set__width(10000);
        sensor_msgs::PointCloud2Modifier modifier(_pc2);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(_pc2.height * _pc2.width);

        sensor_msgs::PointCloud2Iterator<float> iter_x(_pc2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(_pc2, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(_pc2, "z");

        for (int y=0; y < height; y++){
            for(int x = 0; x < width; x++){
                *iter_x = x;
                *iter_y = y;
                if (x < 50 && y < 50) {
                    *iter_z = 3.0;
                }
                else{
                    *iter_z = 1.0;
                }
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
    }

    void republish()
    {
        _publisher->publish(std::move(_pc2));
    }

    void printPC2(){
//TODO
    }

private:
    sensor_msgs::msg::PointCloud2 _pc2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

};

class TestNode :public ::testing::Test
{
public:
    TestNode() {
        map_pub_ = std::make_shared<PC2Publisher>();
    }
    ~TestNode() {}
    void addLayer();
    void publish();
private:
    std::shared_ptr<nav2_util::LifecycleNode> node_;
    std::shared_ptr<PC2Publisher> map_pub_;
};

void TestNode::addLayer() {
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
    tf2_ros::Buffer tf(node_->get_clock());
    nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
    layers.resizeMap(10,10, 0.05, 0.0, 0.0);
    std::shared_ptr<nav2_costmap_2d::StaticLayer3D> nlayer = std::make_shared<nav2_costmap_2d::StaticLayer3D>();
    nlayer->initialize(&layers, "static3d", &tf, node_, nullptr, nullptr /*TODO*/);

    while (!nlayer->receivedMap()){
        map_pub_->republish();
        rclcpp::spin_some(node_->get_node_base_interface());
    }
    layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(nlayer));

//    addObservation(nlayer, 0.0, 0.0, 1.0 / 2, 0, 0, 1.0 / 2);
    layers.updateMap(0, 0, 0);
    nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
    printMap(*costmap);
}

void TestNode::publish() {
    RCLCPP_INFO(
            rclcpp::get_logger("test_node"),
            "publishing"
    );
    map_pub_->republish();
    int b = 0;
}

TEST_F(TestNode, testTemplate){
// or through singlethreadexecutor?
    publish();
    addLayer();


}
//TODO static layer+nlayer
