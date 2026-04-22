#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_segmentation/action/segment_image.hpp"

namespace nav2_segmentation
{

class SegmentationServer : public rclcpp::Node
{
public:
  using SegmentImage = nav2_segmentation::action::SegmentImage;
  using GoalHandleSegmentImage = rclcpp_action::ServerGoalHandle<SegmentImage>;

  SegmentationServer()
  : Node("nav2_segmentation")
  {
    declare_parameter<std::string>("action_name", "segment_image");
    declare_parameter<std::string>("default_mask_topic", "/segmentation/mask");

    const auto action_name = get_parameter("action_name").as_string();
    default_mask_topic_ = get_parameter("default_mask_topic").as_string();

    action_server_ = rclcpp_action::create_server<SegmentImage>(
      this,
      action_name,
      std::bind(&SegmentationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SegmentationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SegmentationServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Started dummy C++ segmentation action server on %s", action_name.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const SegmentImage::Goal> goal)
  {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSegmentImage>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSegmentImage> goal_handle)
  {
    auto feedback = std::make_shared<SegmentImage::Feedback>();
    feedback->current_state = "dummy_complete";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<SegmentImage::Result>();
    result->success = true;
    result->mask_topic = default_mask_topic_;
    result->message = "Dummy C++ segmentation result for discussion.";
    goal_handle->succeed(result);
  }

  std::string default_mask_topic_;
  rclcpp_action::Server<SegmentImage>::SharedPtr action_server_;
};

}  // namespace nav2_segmentation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_segmentation::SegmentationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
