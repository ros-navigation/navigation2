#ifndef NAV2_SEGMENTATION__SEGMENT_IMAGE_ACTION_HPP_
#define NAV2_SEGMENTATION__SEGMENT_IMAGE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_segmentation/action/segment_image.hpp"

namespace nav2_segmentation
{

class SegmentImageAction
  : public nav2_behavior_tree::BtActionNode<nav2_segmentation::action::SegmentImage>
{
  using Action = nav2_segmentation::action::SegmentImage;

public:
  SegmentImageAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("image_topic", "/camera/image", "Input image topic"),
        BT::InputPort<std::string>("prompt", "", "Segmentation prompt"),
        BT::InputPort<bool>("use_latest_image", true, "Use latest image"),
        BT::OutputPort<bool>("success", "Segmentation action success"),
        BT::OutputPort<std::string>("mask_topic", "Output mask topic"),
        BT::OutputPort<std::string>("message", "Segmentation status message")
      });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void on_timeout() override;
};

}  // namespace nav2_segmentation

#endif  // NAV2_SEGMENTATION__SEGMENT_IMAGE_ACTION_HPP_
