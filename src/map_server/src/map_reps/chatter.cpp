#include <iostream>
#include <memory>
#include <cinttypes>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"



/* class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const std::string & service_name)
  : Node("add_two_ints_server")
  {
    // Create a callback function for when service requests are received.
    auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
          request->a, request->b);
        response->sum = request->a + request->b;
      };

    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<example_interfaces::srv::AddTwoInts>(service_name, handle_add_two_ints);
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};



 */













class Chatter
{
public:

    Chatter()
    {  
        node = rclcpp::Node::make_shared("listener");

        std::cout << "Construct Me!" << std::endl;
        
        /* auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", std::bind(&Chatter::chatterCallback, this,
        std::placeholders::_1), rmw_qos_profile_default); */

/*         auto handle_add_two_ints =
            //[this](const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
            {
            //(void)request_header;
            RCLCPP_INFO(node->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
                request->a, request->b);
            response->sum = request->a + request->b;
            }; */

        auto handle_add_two_ints = [this](//const std::shared_ptr<rmw_request_id_t> request_header,
const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) -> void
            {
                //(void)request_header;
            RCLCPP_INFO(node->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
                request->a, request->b);
            response->sum = request->a + request->b;
            };




        service = node->create_service<example_interfaces::srv::AddTwoInts>("addints",handle_add_two_ints);
        //auto service = node->create_service<example_interfaces::srv::AddTwoInts>("addints",std::bind(&Chatter::handle_service,this,std::placeholders::_1));
        
        rclcpp::spin(node); 

    }

    void handle_service(
        //const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
            //(void)request_header;
            /* RCLCPP_INFO(
                node->get_logger(),
                "request: %" PRId64 " + %" PRId64, request->a, request->b) */
            response->sum = request->a + request->b;
            
        }

    rclcpp::Node::SharedPtr getNode(){return node;};


  void chatterCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "I heard: [" << msg->data << "]" << std::endl;
    
  }
private:

rclcpp::Node::SharedPtr node;
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  Chatter chatter;  // object to call method on
  //auto node = chatter.getNode();
    //auto service = node->create_service<example_interfaces::srv::AddTwoInts>("addints",std::bind(&Chatter::handle_service,&chatter,std::placeholders::_1));
   // auto service = node->create_service<example_interfaces::srv::AddTwoInts>("addints",handle_service);

  //auto node = chatter.getNode(); 
  /* auto sub = node->create_subscription<std_msgs::msg::String>(
    "chatter", std::bind(&Chatter::chatterCallback, &chatter,
     std::placeholders::_1), rmw_qos_profile_default); */

  return 0;
}