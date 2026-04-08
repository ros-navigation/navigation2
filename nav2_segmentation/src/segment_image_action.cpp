#include <memory>
#include <string>

#include "nav2_segmentation/segment_image_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace nav2_segmentation
{

SegmentImageAction::SegmentImageAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void SegmentImageAction::on_tick()
{
  getInput("image_topic", goal_.image_topic);
  getInput("prompt", goal_.prompt);
  getInput("use_latest_image", goal_.use_latest_image);
}

BT::NodeStatus SegmentImageAction::on_success()
{
  setOutput("success", result_.result->success);
  setOutput("mask_topic", result_.result->mask_topic);
  setOutput("message", result_.result->message);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SegmentImageAction::on_aborted()
{
  setOutput("success", false);
  setOutput("mask_topic", result_.result->mask_topic);
  setOutput("message", result_.result->message);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SegmentImageAction::on_cancelled()
{
  setOutput("success", false);
  setOutput("mask_topic", "");
  setOutput("message", "Segmentation action cancelled.");
  return BT::NodeStatus::SUCCESS;
}

void SegmentImageAction::on_timeout()
{
  setOutput("success", false);
  setOutput("mask_topic", "");
  setOutput("message", "Segmentation action timed out waiting for server response.");
}

}  // namespace nav2_segmentation

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_segmentation::SegmentImageAction>(
        name, "segment_image", config);
    };

  factory.registerBuilder<nav2_segmentation::SegmentImageAction>("SegmentImage", builder);
}
