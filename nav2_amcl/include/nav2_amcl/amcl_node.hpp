/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NAV2_AMCL__AMCL_NODE_HPP_
#define NAV2_AMCL__AMCL_NODE_HPP_

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "nav_msgs/srv/set_map.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/sensors/laser/laser.hpp"
#include "nav2_util/motion_model/motion_model.hpp"
#include "nav2_util/angleutils.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_dynamic_params/dynamic_params_client.hpp"

#define NEW_UNIFORM_SAMPLING 1

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode();
  void savePoseToServer();

private:
  void updatePoseFromServer();
  void checkLaserReceived();
  void requestMap();
  void createMotionModel();
  nav2_util::Laser * createLaserObject();

  // Callbacks
  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void setMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
    std::shared_ptr<nav_msgs::srv::SetMap::Response> res);

  void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
  // Helper functions for laserReceived CB
  bool addNewScanner(
    int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::string & laser_scan_frame_id,
    geometry_msgs::msg::PoseStamped & laser_pose);
  bool shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta);
  bool updateFilter(
    const int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const pf_vector_t & pose);
  void publishParticleCloud(const pf_sample_set_t * set);
  bool getMaxWeightHyp(
    std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
    int & max_weight_hyp);
  void publishAmclPose(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp);
  void calculateMaptoOdomTransform(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps,
    const int & max_weight_hyp);
  void sendMapToOdomTransform(const tf2::TimePoint & transform_expiration);

  void initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  void freeMapDependentMemory();
  map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  void applyInitialPose();

  // Helper to get odometric pose from transform system
  bool getOdomPose(
    geometry_msgs::msg::PoseStamped & pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & t, const std::string & f);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<nav2_dynamic_params::DynamicParamsClient> dynamic_param_client_;
  void initAmclParams();
  void reconfigureCB();

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::SyncParametersClient::SharedPtr parameters_client;
  rclcpp::Node::SharedPtr parameters_node_;

  bool sent_first_transform_;

  tf2::Transform latest_tf_;
  bool latest_tf_valid_;

  // Pose-generating function used to uniformly distribute particles over
  // the map
  static pf_vector_t uniformPoseGenerator(void * arg);

#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices;
#endif

  // Parameter for what odom to use
  std::string odom_frame_id_;

  // Parameter to store latest odom pose
  geometry_msgs::msg::PoseStamped latest_odom_pose_;

  // Parameter for what base to use
  std::string base_frame_id_;
  std::string global_frame_id_;

  bool use_map_topic_;
  bool first_map_only_;

  tf2::TimePoint save_pose_last_time;
  tf2::Duration save_pose_period;

  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

  map_t * map_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> * laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> * laser_scan_filter_;

  std::vector<nav2_util::Laser *> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;

  // Particle filter
  pf_t * pf_;
  double pf_err_;
  double pf_z_;
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  double d_thresh_;
  double a_thresh_;
  int resample_interval_;
  int resample_count_;
  double laser_min_range_;
  double laser_max_range_;

  // Nomotion update control
  // Used to temporarily let amcl update samples even when no motion occurs
  bool m_force_update;

  nav2_util::MotionModel * motionModel_;
  nav2_util::Laser * laser_;
  std::string robot_model_type_;
  std::string sensor_model_type_;


  // Time for tolerance on the published transform,
  // basically defines how long a map->odom transform is good for
  // std::chrono::duration<double> transform_tolerance_;
  tf2::Duration transform_tolerance_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;  // to let amcl update
  // samples without requiring motion
  rclcpp::Service<nav_msgs::srv::SetMap>::SharedPtr set_map_srv_;

  amcl_hyp_t * initial_pose_hyp_;
  bool first_map_received_;

  std::recursive_mutex configuration_mutex_;

  rclcpp::Time last_laser_received_ts_;
  std::chrono::seconds laser_check_interval_;
  rclcpp::TimerBase::SharedPtr check_laser_timer_;

  int max_beams_;
  int min_particles_;
  int max_particles_;

  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;

  double alpha_slow_;
  double alpha_fast_;

  double z_hit_;
  double z_short_;
  double z_max_;
  double z_rand_;
  double sigma_hit_;
  double lambda_short_;

  // Beam skip related params
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;
  double laser_likelihood_max_dist_;

  std::string odom_model_type_;
  std::string laser_model_type_;

  double init_pose_[3];
  double init_cov_[3];

  bool tf_broadcast_;
  int scan_error_count_ = 0;
  bool initial_pose_received;
};

#endif  // NAV2_AMCL__AMCL_NODE_HPP_
