#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "angles/angles.h"

#include "nav2_ceres_costaware_smoother/ceres_costaware_smoother.hpp"

using namespace nav2_ceres_costaware_smoother;

class DummyCostmapSubscriber : public nav2_costmap_2d::CostmapSubscriber
{
public:
  DummyCostmapSubscriber(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::string & topic_name)
  : CostmapSubscriber(node, topic_name)
  {
    auto costmap = std::make_shared<nav2_msgs::msg::Costmap>();
    costmap->metadata.size_x = 100;
    costmap->metadata.size_y = 100;
    costmap->metadata.resolution = 0.1;
    costmap->metadata.origin.position.x = -5.0;
    costmap->metadata.origin.position.y = -5.0;

    costmap->data.resize(costmap->metadata.size_x * costmap->metadata.size_y, 0);

    // create an obstacle in rect (1.0, -1.0) -> (3.0, -2.0)
    // with inflation of radius 2.0
    double cost_scaling_factor = 1.6;
    double inscribed_radius = 0.3;
    for (int i = 10; i < 60; ++i) {
      for (int j = 40; j < 100; ++j) {
        int dist_x = std::max(0, std::max(60 - j, j - 80));
        int dist_y = std::max(0, std::max(30 - i, i - 40));
        double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
        unsigned char cost;
        if (dist == 0)
          cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        else if (dist < inscribed_radius)
          cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        else {
          double factor =
            exp(-1.0 * cost_scaling_factor * (dist * costmap->metadata.resolution - inscribed_radius));
          cost = static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        costmap->data[i * costmap->metadata.size_x + j] = cost;
      }
    }

    setCostmap(costmap);
  }

  void setCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    costmap_msg_ = msg;
    costmap_received_ = true;
  }
};

class SmootherTest : public ::testing::Test
{
protected:
  SmootherTest() {SetUp();}
  ~SmootherTest() {}

  void SetUp()
  {
    node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleRecoveryTestNode", rclcpp::NodeOptions());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    costmap_sub_ =
      std::make_shared<DummyCostmapSubscriber>(
      node_lifecycle_, "costmap_topic");

    smoother_ = std::make_shared<CeresCostawareSmoother>();
    smoother_->configure(
      node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();
  }

  void TearDown() override
  {
    smoother_->deactivate();
    smoother_->cleanup();
  }

  void reloadParams()
  {
    smoother_->deactivate();
    smoother_->cleanup();
    smoother_->configure(
      node_lifecycle_, "SmoothPath", tf_buffer_, costmap_sub_,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>());
    smoother_->activate();
  }

  bool smoothPath(const std::vector<Eigen::Vector3d> &input, std::vector<Eigen::Vector3d> &output)
  {
    nav_msgs::msg::Path path;
    path.poses.reserve(input.size());
    for (auto &xya : input) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = xya.x();
      pose.pose.position.y = xya.y();
      pose.pose.position.z = 0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(xya.z());
      path.poses.push_back(pose);
    }

    bool result = smoother_->smooth(path, rclcpp::Duration::from_seconds(1.0));
    
    output.clear();
    output.reserve(path.poses.size());
    for (auto &pose : path.poses) {
      Eigen::Vector3d xya;
      xya.x() = pose.pose.position.x;
      xya.y() = pose.pose.position.y;
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      xya.z() = q.getAngle();
      output.push_back(xya);
    }
    return result;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sum(criterion(data[i])^2)/count(data)
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> &input,
    const std::vector<Eigen::Vector3d> &output,
    const std::function<double(int i, const Eigen::Vector3d &prev_p, const Eigen::Vector3d &p, const Eigen::Vector3d &next_p)> &criterion)
  {
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size()-1; i++) {
      double input_crit = criterion(i, input[i-1], input[i], input[i+1]);
      total_input_crit += input_crit*input_crit;
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "input p: %lf %lf, c: %lf, total: %lf", input[i][0], input[i][1], input_crit, total_input_crit);
    }
    total_input_crit /= input.size() - 2;

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size()-1; i++) {
      double output_crit = criterion(i, output[i-1], output[i], output[i+1]);
      total_output_crit += output_crit*output_crit;
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "output p: %lf %lf, c: %lf, total: %lf", output[i][0], output[i][1], output_crit, total_output_crit);
    }
    total_output_crit /= output.size() - 2;

    return (1.0 - total_output_crit/total_input_crit)*100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sum(criterion(data[i])^2)/count(data)
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> &input,
    const std::vector<Eigen::Vector3d> &output,
    const std::function<double(int i, const Eigen::Vector3d &prev_p, const Eigen::Vector3d &p)> &criterion)
  {
    double total_input_crit = 0.0;
    for (size_t i = 1; i < input.size(); i++) {
      double input_crit = criterion(i, input[i-1], input[i]);
      total_input_crit += input_crit*input_crit;
    }
    total_input_crit /= input.size() - 1;

    double total_output_crit = 0.0;
    for (size_t i = 1; i < output.size(); i++) {
      double output_crit = criterion(i, output[i-1], output[i]);
      total_output_crit += output_crit*output_crit;
    }
    total_output_crit /= output.size() - 1;

    return (1.0 - total_output_crit/total_input_crit)*100;
  }

  /**
   * @brief Path improvement assessment method
   * @param input Smoother input path
   * @param output Smoother output path
   * @param criterion Criterion of path quality. Total path quality = sum(criterion(data[i])^2)/count(data)
   * @return Percentage of improvement (relative to input path quality)
   */
  double assessPathImprovement(
    const std::vector<Eigen::Vector3d> &input,
    const std::vector<Eigen::Vector3d> &output,
    const std::function<double(int i, const Eigen::Vector3d &p)> &criterion)
  {
    double total_input_crit = 0.0;
    for (size_t i = 0; i < input.size(); i++) {
      double input_crit = criterion(i, input[i]);
      total_input_crit += input_crit*input_crit;
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "input p: %lf %lf, c: %lf, total: %lf", input[i][0], input[i][1], input_crit, total_input_crit);
    }
    total_input_crit /= input.size();

    double total_output_crit = 0.0;
    for (size_t i = 0; i < output.size(); i++) {
      double output_crit = criterion(i, output[i]);
      total_output_crit += output_crit*output_crit;
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "output p: %lf %lf, c: %lf, total: %lf", output[i][0], output[i][1], output_crit, total_output_crit);
    }
    total_output_crit /= output.size();

    return (1.0 - total_output_crit/total_input_crit)*100;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<nav2_ceres_costaware_smoother::CeresCostawareSmoother> smoother_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<DummyCostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
};

TEST_F(SmootherTest, testingSmoothness)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  // set w_curve to 0.0 so that the whole job is upon w_smooth
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_dir_change", 0.0));
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.3, 0, M_PI/4}
    , {0.3, 0.1, M_PI/2}
    , {0.3, 0.2, M_PI/2}
    , {0.3, 0.3, M_PI/2}
    };
  int cusp_i = -1;

  std::vector<Eigen::Vector3d> smoothed_path;
  ASSERT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  auto mvmt_smoothness_criterion =
    [&cusp_i](int i, const Eigen::Vector3d &prev_p, const Eigen::Vector3d &p, const Eigen::Vector3d &next_p) {
      Eigen::Vector2d prev_mvmt = p.block<2, 1>(0, 0) - prev_p.block<2, 1>(0, 0);
      Eigen::Vector2d next_mvmt = next_p.block<2, 1>(0, 0) - p.block<2, 1>(0, 0);
      if (i == cusp_i)
        next_mvmt = -next_mvmt;
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "prev_mvmt: %lf %lf, next_mvmt: %lf %lf",
        prev_mvmt[0], prev_mvmt[1], next_mvmt[0], next_mvmt[1]);
      return (next_mvmt - prev_mvmt).norm();
    };
  double mvmt_smoothness_improvement =
    assessPathImprovement(sharp_turn_90, smoothed_path, mvmt_smoothness_criterion);
  ASSERT_GT(mvmt_smoothness_improvement, 0.0);
  ASSERT_NEAR(mvmt_smoothness_improvement, 80.0, 1.0);

  auto orientation_smoothness_criterion =
    [] (int, const Eigen::Vector3d &prev_p, const Eigen::Vector3d &p) {
      return angles::normalize_angle(p.z() - prev_p.z());
    };
  double orientation_smoothness_improvement =
    assessPathImprovement(sharp_turn_90, smoothed_path, orientation_smoothness_criterion);
  ASSERT_GT(orientation_smoothness_improvement, 0.0);
  ASSERT_NEAR(orientation_smoothness_improvement, 62.0, 1.0);

  // path with a cusp
  std::vector<Eigen::Vector3d> sharp_turn_90_then_reverse =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.3, 0, 0}
    , {0.4, 0, 0}
    , {0.5, 0, 0}
    , {0.6, 0, M_PI/4}
    , {0.6, -0.1, M_PI/2}
    , {0.6, -0.2, M_PI/2}
    , {0.6, -0.3, M_PI/2}
    , {0.6, -0.4, M_PI/2}
    , {0.6, -0.5, M_PI/2}
    , {0.6, -0.6, M_PI/2}
    };
  cusp_i = 6;

  ASSERT_TRUE(smoothPath(sharp_turn_90_then_reverse, smoothed_path));

  mvmt_smoothness_improvement =
    assessPathImprovement(sharp_turn_90_then_reverse, smoothed_path, mvmt_smoothness_criterion);
  ASSERT_GT(mvmt_smoothness_improvement, 0.0);
  ASSERT_NEAR(mvmt_smoothness_improvement, 60.5, 1.0);

  orientation_smoothness_improvement =
    assessPathImprovement(sharp_turn_90_then_reverse, smoothed_path, orientation_smoothness_criterion);
  ASSERT_GT(orientation_smoothness_improvement, 0.0);
  ASSERT_NEAR(orientation_smoothness_improvement, 48.9, 1.0);

  SUCCEED();
}

TEST_F(SmootherTest, testingAnchoringToOriginalPath)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 30.0));
  // set w_curve to 0.0 so that the whole job is upon w_smooth
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_dir_change", 0.0));

  // first set w_dist to 0.0 to generate an unanchored smoothed path
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 0.0));
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.3, 0, M_PI/4}
    , {0.3, 0.1, M_PI/2}
    , {0.3, 0.2, M_PI/2}
    , {0.3, 0.3, M_PI/2}
    };

  std::vector<Eigen::Vector3d> smoothed_path;
  ASSERT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  // then update w_dist and compare the results
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 30.0));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_anchored;
  ASSERT_TRUE(smoothPath(sharp_turn_90, smoothed_path_anchored));

  auto origin_similarity_criterion =
    [&sharp_turn_90](int i, const Eigen::Vector3d &p) {
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "orig: %lf %lf", sharp_turn_90[i][0], sharp_turn_90[i][1]);
      return (p.block<2, 1>(0, 0) - sharp_turn_90[i].block<2, 1>(0, 0)).norm();
    };
  double origin_similarity_improvement =
    assessPathImprovement(smoothed_path, smoothed_path_anchored, origin_similarity_criterion);
  ASSERT_GT(origin_similarity_improvement, 0.0);
  ASSERT_NEAR(origin_similarity_improvement, 70.3, 1.0);
}

TEST_F(SmootherTest, testingMaxCurvature)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.minimum_turning_radius", 0.4));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 30.0));
  // set w_smooth to a small value so that the whole job is upon w_curve
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 0.3));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_dir_change", 0.0));
  reloadParams();

  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  std::vector<Eigen::Vector3d> radius_0_3_turn_90 =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.2 + 0.3*sin(M_PI/12), 0.3*(1 - cos(M_PI/12)), 0}
    , {0.2 + 0.3*sin(M_PI*2/12), 0.3*(1 - cos(M_PI*2/12)), 0}
    , {0.2 + 0.3*sin(M_PI*3/12), 0.3*(1 - cos(M_PI*3/12)), 0}
    , {0.2 + 0.3*sin(M_PI*4/12), 0.3*(1 - cos(M_PI*4/12)), 0}
    , {0.2 + 0.3*sin(M_PI*5/12), 0.3*(1 - cos(M_PI*5/12)), 0}
    , {0.5, 0.3, M_PI/2}
    , {0.5, 0.4, M_PI/2}
    , {0.5, 0.5, M_PI/2}
    };

  std::vector<Eigen::Vector3d> smoothed_path;
  ASSERT_TRUE(smoothPath(radius_0_3_turn_90, smoothed_path));

  // smoothed path points should form a circle with radius 0.4
  for (size_t i = 1; i < smoothed_path.size()-1; i++) {
    auto &p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(0.1, 0.4)).norm();
    ASSERT_NEAR(r, 0.4, 0.01);
  }

  // path with a cusp
  // smoother should increase radius from infeasible 0.3 to feasible 0.4
  std::vector<Eigen::Vector3d> radius_0_3_turn_90_then_reverse_turn_90 =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.2 + 0.3*sin(M_PI/12), 0.3*(1 - cos(M_PI/12)), M_PI/12}
    , {0.2 + 0.3*sin(M_PI*2/12), 0.3*(1 - cos(M_PI*2/12)), M_PI*2/12}
    , {0.2 + 0.3*sin(M_PI*3/12), 0.3*(1 - cos(M_PI*3/12)), M_PI*3/12}
    , {0.2 + 0.3*sin(M_PI*4/12), 0.3*(1 - cos(M_PI*4/12)), M_PI*4/12}
    , {0.2 + 0.3*sin(M_PI*5/12), 0.3*(1 - cos(M_PI*5/12)), M_PI*5/12}
    , {0.5, 0.3, M_PI/2}
    , {0.8 - 0.3*sin(M_PI*5/12), 0.3*(1 - cos(M_PI*5/12)), M_PI*7/12}
    , {0.8 - 0.3*sin(M_PI*4/12), 0.3*(1 - cos(M_PI*4/12)), M_PI*8/12}
    , {0.8 - 0.3*sin(M_PI*3/12), 0.3*(1 - cos(M_PI*3/12)), M_PI*9/12}
    , {0.8 - 0.3*sin(M_PI*2/12), 0.3*(1 - cos(M_PI*2/12)), M_PI*10/12}
    , {0.8 - 0.3*sin(M_PI/12), 0.3*(1 - cos(M_PI/12)), M_PI*11/12}
    , {0.8, 0, M_PI}
    , {0.9, 0, M_PI}
    , {1.0, 0, M_PI}
    };

  ASSERT_TRUE(smoothPath(radius_0_3_turn_90_then_reverse_turn_90, smoothed_path));

  // smoothed path points should form a circle with radius 0.4 for both forward and reverse movements
  for (size_t i = 1; i < smoothed_path.size()-1; i++) {
    auto &p = smoothed_path[i];
    double r = (p.block<2, 1>(0, 0) - Eigen::Vector2d(i <= 8 ? 0.1 : 0.9, 0.4)).norm();
    ASSERT_NEAR(r, 0.4, 0.01);
  }

  SUCCEED();
}

TEST_F(SmootherTest, testingObstacleAvoidance)
{
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_smooth", 15000.0));
  // set w_curve to 0.0 so that the whole job is upon w_smooth
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_curve", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost", 0.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_cost_dir_change", 0.0));

  // first set w_dist to 0.0 to generate an unanchored smoothed path
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 0.0));
  reloadParams();

  std::vector<Eigen::Vector3d> sharp_turn_90 =
    { {0, 0, 0}
    , {0.1, 0, 0}
    , {0.2, 0, 0}
    , {0.3, 0, M_PI/4}
    , {0.3, 0.1, M_PI/2}
    , {0.3, 0.2, M_PI/2}
    , {0.3, 0.3, M_PI/2}
    };

  std::vector<Eigen::Vector3d> smoothed_path;
  ASSERT_TRUE(smoothPath(sharp_turn_90, smoothed_path));

  // then update w_dist and compare the results
  node_lifecycle_->set_parameter(rclcpp::Parameter("SmoothPath.w_dist", 30.0));
  reloadParams();

  std::vector<Eigen::Vector3d> smoothed_path_anchored;
  ASSERT_TRUE(smoothPath(sharp_turn_90, smoothed_path_anchored));

  auto origin_similarity_criterion =
    [&sharp_turn_90](int i, const Eigen::Vector3d &p) {
      RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), "orig: %lf %lf", sharp_turn_90[i][0], sharp_turn_90[i][1]);
      return (p.block<2, 1>(0, 0) - sharp_turn_90[i].block<2, 1>(0, 0)).norm();
    };
  double origin_similarity_improvement =
    assessPathImprovement(smoothed_path, smoothed_path_anchored, origin_similarity_criterion);
  ASSERT_GT(origin_similarity_improvement, 0.0);
  ASSERT_NEAR(origin_similarity_improvement, 70.3, 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
