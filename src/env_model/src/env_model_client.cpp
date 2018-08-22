#include "rclcpp/rclcpp.hpp"
#include "env_model_msgs/srv/float_to_int.hpp"
#include "env_model_msgs/srv/int_to_float.hpp"
#include "env_model_msgs/srv/get_value_at.hpp"
#include "env_model_msgs/srv/get_value_at_pos.hpp"
#include "env_model_msgs/srv/is_map_valid.hpp"
#include "env_model_msgs/srv/get_ray_trace.hpp"
#include "env_model_msgs/srv/get_map.hpp"

rclcpp::Node::SharedPtr node = nullptr;


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


int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  node = rclcpp::Node::make_shared("env_client");


  // Test Clients //


  auto Cell_client = node->create_client<env_model_msgs::srv::FloatToInt>("getCell");
  auto Cell_request = std::make_shared<env_model_msgs::srv::FloatToInt::Request>();
  Cell_request->d = {5,5};
  auto Cell_response = send_request <env_model_msgs::srv::FloatToInt> (node,Cell_client,Cell_request);
  RCLCPP_INFO(node->get_logger(), "index: [%ld, %ld]", Cell_response->i[0], Cell_response->i[1]);


  auto Pos_client = node->create_client<env_model_msgs::srv::IntToFloat>("getPos");
  auto Pos_request = std::make_shared<env_model_msgs::srv::IntToFloat::Request>();
  Pos_request->i = {2,3};
  auto Pos_response = send_request <env_model_msgs::srv::IntToFloat> (node,Pos_client,Pos_request);
  RCLCPP_INFO(node->get_logger(), "pos: [%f, %f]", Pos_response->d[0], Pos_response->d[1]);


  auto ValueAt_client = node->create_client<env_model_msgs::srv::GetValueAt>("getValueAt");
  auto ValueAt_request = std::make_shared<env_model_msgs::srv::GetValueAt::Request>();
  ValueAt_request->layer = "distance";
  ValueAt_request->index = {5,5};
  auto ValueAt_response = send_request <env_model_msgs::srv::GetValueAt> (node,ValueAt_client,ValueAt_request);
  RCLCPP_INFO(node->get_logger(), "value: %f", ValueAt_response->value);


  auto ValueAtPos_client = node->create_client<env_model_msgs::srv::GetValueAtPos>("getValueAtPos");
  auto ValueAtPos_request = std::make_shared<env_model_msgs::srv::GetValueAtPos::Request>();
  ValueAtPos_request->layer = "static";
  ValueAtPos_request->pos = {2,3};
  auto ValueAtPos_response = send_request <env_model_msgs::srv::GetValueAtPos> (node,ValueAtPos_client,ValueAtPos_request);
  RCLCPP_INFO(node->get_logger(), "value: %f", ValueAtPos_response->value);


  auto IsValid_client = node->create_client<env_model_msgs::srv::IsMapValid>("isValid");
  auto IsValid_request = std::make_shared<env_model_msgs::srv::IsMapValid::Request>();
  IsValid_request->index = {5,5};
  auto IsValid_response = send_request <env_model_msgs::srv::IsMapValid> (node,IsValid_client,IsValid_request);
  RCLCPP_INFO(node->get_logger(), "valid: %d", IsValid_response->isvalid);


  auto RayTrace_client = node->create_client<env_model_msgs::srv::GetRayTrace>("rayTrace");
  auto RayTrace_request = std::make_shared<env_model_msgs::srv::GetRayTrace::Request>();
  RayTrace_request->x = 5;
  RayTrace_request->y = 5;
  RayTrace_request->a = 0;
  RayTrace_request->max = 100;
  auto RayTrace_response = send_request <env_model_msgs::srv::GetRayTrace> (node,RayTrace_client,RayTrace_request);
  RCLCPP_INFO(node->get_logger(), "range: %f", RayTrace_response->range);


  auto Map_client = node->create_client<env_model_msgs::srv::GetMap>("getMap");
  auto Map_request = std::make_shared<env_model_msgs::srv::GetMap::Request>();
  Map_request->layer = "static";
  auto Map_response = send_request <env_model_msgs::srv::GetMap> (node,Map_client,Map_request);

  
}
