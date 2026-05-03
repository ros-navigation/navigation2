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
        BT::InputPort<std::string>("text_prompt", "", "Text prompt"),
        BT::InputPort<float>("point_x", 0.0F, "Point prompt x coordinate"),
        BT::InputPort<float>("point_y", 0.0F, "Point prompt y coordinate"),
        BT::InputPort<int>("point_label", 1, "Point label (1 foreground, 0 background)"),
        BT::InputPort<float>("box_min_x", 0.0F, "Box minimum x"),
        BT::InputPort<float>("box_min_y", 0.0F, "Box minimum y"),
        BT::InputPort<float>("box_max_x", 0.0F, "Box maximum x"),
        BT::InputPort<float>("box_max_y", 0.0F, "Box maximum y"),
        BT::InputPort<bool>("use_point_prompt", false, "Use point prompt"),
        BT::InputPort<bool>("use_box_prompt", false, "Use box prompt"),
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
