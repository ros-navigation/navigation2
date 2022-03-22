// Copyright 2022 FastSense, Samsung Research
#include "mppic/path_handler.hpp"

#include "mppic/utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace mppi
{

using nav2_util::geometry_utils::euclidean_distance;

void PathHandler::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
  std::shared_ptr<tf2_ros::Buffer> buffer)
{
  name_ = name;
  costmap_ = costmap;
  tf_buffer_ = buffer;
  auto node = parent.lock();
  logger_ = node->get_logger();

  auto getParam = utils::getParamGetter(node, name_);
  getParam(max_robot_pose_search_dist_, "max_robot_pose_search_dist", getMaxCostmapDist());
  getParam(transform_tolerance_, "transform_tolerance", 0.1);
}

PathRange PathHandler::getGlobalPlanConsideringBounds(
  const geometry_msgs::msg::PoseStamped & global_pose)
{
  auto begin = global_plan_.poses.begin();
  auto end = global_plan_.poses.end();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // Find closest point to the robot

  auto closest_point = nav2_util::geometry_utils::min_by(
    begin, closest_pose_upper_bound,
    [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(global_pose, ps);
    });

  // Find the furthest relevent point on the path to consider within costmap
  // bounds
  auto max_costmap_dist = getMaxCostmapDist();

  auto last_point =
    std::find_if(
    closest_point, end, [&](const geometry_msgs::msg::PoseStamped & global_plan_pose) {
      return euclidean_distance(global_pose, global_plan_pose) > max_costmap_dist;
    });

  return {closest_point, last_point};
}

geometry_msgs::msg::PoseStamped PathHandler::transformToGlobalPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error(
            "Unable to transform robot pose into global plan's frame");
  }

  return robot_pose;
}

nav_msgs::msg::Path PathHandler::transformPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find relevent bounds of path to use
  geometry_msgs::msg::PoseStamped global_pose =
    transformToGlobalPlanFrame(robot_pose);
  auto [lower_bound, upper_bound] = getGlobalPlanConsideringBounds(global_pose);

  // Transform these bounds into the local costmap frame and prune older points
  const auto & stamp = global_pose.header.stamp;
  nav_msgs::msg::Path transformed_plan =
    transformPlanPosesToCostmapFrame(lower_bound, upper_bound, stamp);

  pruneGlobalPlan(lower_bound);

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_buffer_->transform(
      in_pose, out_pose, frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double PathHandler::getMaxCostmapDist()
{
  const auto & costmap = costmap_->getCostmap();
  return std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
         costmap->getResolution() / 2.0;
}

nav_msgs::msg::Path PathHandler::transformPlanPosesToCostmapFrame(
  PathIterator begin, PathIterator end, const StampType & stamp)
{
  std::string frame = costmap_->getGlobalFrameID();
  auto transformToFrame = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped from_pose;
      geometry_msgs::msg::PoseStamped to_pose;

      from_pose.header.frame_id = global_plan_.header.frame_id;
      from_pose.header.stamp = stamp;
      from_pose.pose = global_plan_pose.pose;

      transformPose(frame, from_pose, to_pose);
      return to_pose;
    };

  nav_msgs::msg::Path plan;
  plan.header.frame_id = frame;
  plan.header.stamp = stamp;

  std::transform(begin, end, std::back_inserter(plan.poses), transformToFrame);

  return plan;
}

void PathHandler::setPath(const nav_msgs::msg::Path & plan)
{
  global_plan_ = plan;
}

nav_msgs::msg::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::pruneGlobalPlan(const PathIterator end)
{
  global_plan_.poses.erase(global_plan_.poses.begin(), end);
}

}  // namespace mppi
