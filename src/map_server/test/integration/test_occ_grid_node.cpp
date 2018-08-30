#include <gtest/gtest.h>
#include "map_server/map_reps/occupancy_grid.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "test_constants.h"

// rclcpp::init can only be called once per process, so this needs to be a global variable
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    n_ = rclcpp::Node::make_shared("map_client_test");

  }

template <class T>
typename T::Response::SharedPtr send_request(

    rclcpp::Node::SharedPtr node,
    typename rclcpp::Client<T>::SharedPtr client,
    typename T::Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
    }

}

protected:
  rclcpp::Node::SharedPtr n_;
  //bool got_map_metadata_;
  //bool got_map_metadata_;

};

TEST_F(TestNode, ResultReturned)
{
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto client = n_->create_client<nav_msgs::srv::GetMap>("static_occ_grid");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto resp = send_request<nav_msgs::srv::GetMap> (n_,client,req);
  //ASSERT_TRUE(ros::service::call("static_map", req, resp));
  ASSERT_FLOAT_EQ(resp->map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp->map.info.width, g_valid_image_width);
  ASSERT_EQ(resp->map.info.height, g_valid_image_height);
  //ASSERT_STREQ(resp->map.header.frame_id.c_str(), "map");
  for(unsigned int i=0; i < resp->map.info.width * resp->map.info.height; i++)
    ASSERT_EQ(g_valid_image_content[i], resp->map.data[i]);

}