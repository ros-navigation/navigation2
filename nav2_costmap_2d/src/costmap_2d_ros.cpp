/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <nav2_costmap_2d/layered_costmap.h>
#include <nav2_costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav2_util/duration_conversions.h"

using namespace std;

namespace nav2_costmap_2d
{

void move_parameter(rclcpp::Node::SharedPtr old_h, rclcpp::Node::SharedPtr new_h, std::string name,
    bool should_delete = true)
{
  auto parameters_client_old = std::make_shared<rclcpp::SyncParametersClient>(old_h);
  auto parameters_client_new = std::make_shared<rclcpp::SyncParametersClient>(new_h);

  if (!parameters_client_old->has_parameter(name)) {
    return;
  }

  std::vector<rclcpp::Parameter> param;
  param = parameters_client_old->get_parameters({name});
  parameters_client_new->set_parameters(param);
  //TODO(bpwilcox): port delete parameter from client?
  //if (should_delete) old_h.deleteParam(name);
}

Costmap2DROS::Costmap2DROS(const std::string & name, tf2_ros::Buffer & tf)
  : layered_costmap_(NULL),
  name_(name),
  tf_(tf),
  transform_tolerance_(10000),
  map_update_thread_shutdown_(false),
  stop_updates_(false),
  initialized_(true),
  stopped_(false),
  robot_stopped_(false),
  map_update_thread_(NULL),
  last_publish_(0),
  plugin_loader_("nav2_costmap_2d", "nav2_costmap_2d::Layer"),
  publisher_(NULL),
  publish_cycle_(1),
  footprint_padding_(0.0)
{
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  auto private_nh = rclcpp::Node::make_shared(name_);
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  // get two frames
  auto private_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node);

  this->get_parameter_or<std::string>("global_frame", global_frame_, std::string("map"));
  this->get_parameter_or<std::string>("robot_base_frame", robot_base_frame_, std::string("base_link"));

  rclcpp::Clock clock;
  rclcpp::Time last_error = clock.now();
  std::string tf_error;

  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (rclcpp::ok() &&
      !tf_.canTransform(global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1), &tf_error))
  {
    //rclcpp::spin_some(private_nh);
    if (last_error + nav2_util::durationFromSeconds(5.0) < clock.now()) {
      RCLCPP_WARN(rclcpp::get_logger(
            "nav2_costmap_2d"),
          "Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
          robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = private_nh_->now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  this->get_parameter_or<bool>("rolling_window", rolling_window, false);
  this->get_parameter_or<bool>("track_unknown_space", track_unknown_space, false);
  this->get_parameter_or<bool>("always_send_full_costmap", always_send_full_costmap, false);


  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  std::vector<rclcpp::Parameter> param;
  rclcpp::Parameter temp;
  if (!this->get_parameter("plugin_names", temp)) {
  setPluginParams(temp_node);
  }

  if (this->get_parameter("plugin_names", temp)) {
  //if (private_parameters_client->has_parameter("plugin_names")) {
    param = this->get_parameters({"plugin_names", "plugin_types"});

    for (int32_t i = 0; i < param[0].get_value<std::vector<std::string>>().size(); ++i) {
      std::string pname = (param[0].get_value<std::vector<std::string>>())[i];
      std::string type = (param[1].get_value<std::vector<std::string>>())[i];

      RCLCPP_INFO(rclcpp::get_logger("costmap_2d"), "Using plugin \"%s\"", pname.c_str());

      std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(type);
      layered_costmap_->addPlugin(plugin);
      plugin->initialize(layered_costmap_, name + "_" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  std::string topic_param, topic;
  this->get_parameter_or<std::string>("footprint_topic", topic_param, std::string("footprint_topic"));
  this->get_parameter_or<std::string>(topic_param, topic, std::string("footprint"));
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::Polygon>(topic,
      std::bind(&Costmap2DROS::setUnpaddedRobotFootprintPolygon, this, std::placeholders::_1));
  this->get_parameter_or<std::string>("published_footprint_topic", topic_param, std::string("published_footprint"));
  this->get_parameter_or<std::string>(topic_param, topic, std::string("oriented_footprint"));

  footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "footprint", rmw_qos_profile_default);

  setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh_));

  publisher_ = new Costmap2DPublisher(private_nh_,
      layered_costmap_->getCostmap(), global_frame_, "costmap",
      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a timer to check if the robot is moving
  robot_stopped_ = false;
  timer_ = private_nh_->create_wall_timer(100ms, std::bind(&Costmap2DROS::movementCB, this));

  // TODO(bpwilcox): resolve dynamic reconfigure dependencies
  this->register_param_change_callback(std::bind(&Costmap2DROS::on_param_event, this, std::placeholders::_1));
  parameter_sub_ = private_parameters_client->on_parameter_event(std::bind(&Costmap2DROS::parameter_event_callback, this, std::placeholders::_1));

  auto set_parameters_results = this->set_parameters({
    rclcpp::Parameter("update_frequency", 1.0),
    //rclcpp::Parameter("transform_tolerance", 0.3),
    //rclcpp::Parameter("publish_frequency", 0.0),
    //rclcpp::Parameter("width", 10),
    //rclcpp::Parameter("height", 10),
    //rclcpp::Parameter("resolution", 0.1),
    //rclcpp::Parameter("origin_x", 0.0),
    //rclcpp::Parameter("origin_y", 0.0),
    //rclcpp::Parameter("footprint", "[]"),
    //rclcpp::Parameter("robot_radius", 0.46),
    //rclcpp::Parameter("footprint_padding", 0.01)
  });

  //this->set_parameter_if_not_set <double> ("transform_tolerance", transform_tolerance_);


bool validate_param(std::string param_name, std::map<std::string, rclcpp::Parameter> & map, rclcpp::ParameterType Type)
{
  if (map.count(param_name) > 0)
    if(Type == map[param_name].get_type()){
      return true;
    }
    else{
      return false;
    }
  return true;
}



rcl_interfaces::msg::SetParametersResult Costmap2DROS::on_param_event(
  std::vector<rclcpp::Parameter> parameters)
{

  std::map<std::string, rclcpp::Parameter> param_map;

  for (auto parameter : parameters){
  param_map[parameter.get_name()] = parameter;
  RCLCPP_INFO(this->get_logger(), "parameter change: %s", (parameter.get_name()).c_str());
  }

  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  if(get_param(
    "transform_tolerance", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, transform_tolerance_)){
    result.successful &= true;
  } else {result.successful = false;}
  
/*   result.successful &= get_param(
    "transform_tolerance", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, transform_tolerance_); */

  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;

  double map_update_frequency;
  if(get_param(
    "update_frequency", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, map_update_frequency)){
    result.successful &= true;
  } else {result.successful = false;}
/*   result.successful &= get_param(
    "update_frequency", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, map_update_frequency); */

 double map_publish_frequency;
  if(get_param(
    "publish_frequency", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, map_publish_frequency)){
    result.successful &= true;
  } else {result.successful = false;}
/*   result.successful &= get_param(
    "publish_frequency", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, map_publish_frequency); */

  if (map_publish_frequency > 0)
    publish_cycle_ = rclcpp::Duration(1*1e9 / map_publish_frequency);
  else
    publish_cycle_ = rclcpp::Duration(-1*1e9);

  // find size parameters
  double resolution, origin_x, origin_y;
  int map_width_meters, map_height_meters;
  if(get_param(
    "width", param_map, rclcpp::ParameterType::PARAMETER_INTEGER, map_width_meters)){
    result.successful &= true;
  } else {result.successful = false;}
  if(get_param(
    "height", param_map, rclcpp::ParameterType::PARAMETER_INTEGER, map_height_meters)){
    result.successful &= true;
  } else {result.successful = false;}
  if(get_param(
    "resolution", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, resolution)){
    result.successful &= true;
  } else {result.successful = false;}
  if(get_param(
    "origin_x", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, origin_x)){
    result.successful &= true;
  } else {result.successful = false;}
  if(get_param(
    "origin_y", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, origin_y)){
    result.successful &= true;
  } else {result.successful = false;}

/*   result.successful &= get_param(
    "width", param_map, rclcpp::ParameterType::PARAMETER_INTEGER, map_width_meters);
   result.successful &= get_param(
    "height", param_map, rclcpp::ParameterType::PARAMETER_INTEGER, map_height_meters);
  result.successful = get_param(
    "resolution", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, resolution);
  result.successful &= get_param(
    "origin_x", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, origin_x);
  result.successful &= get_param(
    "origin_y", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, origin_y); */

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  float footprint_padding;
  if(get_param(
    "footprint_padding", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, footprint_padding)){
    result.successful &= true;
  } else {result.successful = false;}
  
/*   result.successful &= get_param(
    "footprint_padding", param_map, rclcpp::ParameterType::PARAMETER_DOUBLE, footprint_padding); */

  if (footprint_padding_ != footprint_padding)
  {  
    footprint_padding_ = footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  map_update_thread_ = new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
  
  return result;
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(
    const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL) {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL) {
    delete publisher_;
  }

  delete layered_costmap_;
  // TODO(bpwilcox): resolve dynamic reconfigure dependencies
  //delete dsrv_;
}

void Costmap2DROS::setPluginParams(rclcpp::Node::SharedPtr nh)
{
  std::vector<rclcpp::Parameter> param;

  std::vector<std::string> plugin_names = {"static_layer","inflation_layer"};
  std::vector<std::string> plugin_types = {"costmap_2d::StaticLayer","costmap_2d::InflationLayer"};
  param = {rclcpp::Parameter("plugin_names",plugin_names),rclcpp::Parameter("plugin_types",plugin_types)};
  nh->set_parameters(param);
}

// TODO(bpwilcox): resolve dynamic reconfigure dependencies
void Costmap2DROS::parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  getParamValue(event,"transform_tolerance", transform_tolerance_);

  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency;
  getParamValue(event,"update_frequency", map_update_frequency);

  double map_publish_frequency;
  getParamValue(event,"publish_frequency", map_publish_frequency);

  if (map_publish_frequency > 0)
    publish_cycle_ = rclcpp::Duration(1*1e9 / map_publish_frequency);
  else
    publish_cycle_ = rclcpp::Duration(-1*1e9);

  // find size parameters
  double resolution, origin_x, origin_y;
  int map_width_meters, map_height_meters;

  getParamValue(event,"width", map_width_meters);
  getParamValue(event,"height", map_height_meters);
  getParamValue(event,"resolution", resolution);
  getParamValue(event,"origin_x", origin_x);
  getParamValue(event,"origin_y", origin_y); 
  
  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  float footprint_padding;
  getParamValue(event,"footprint_padding", footprint_padding);
  if (footprint_padding_ != footprint_padding)
  {  
    footprint_padding_ = footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  //readFootprintFromConfig(config, old_config_);

  //old_config_ = config;
  //RCLCPP_INFO(rclcpp::get_logger("costmap_2d"), "parameter callback: Start Thread?");
  map_update_thread_ = new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
  //RCLCPP_INFO(rclcpp::get_logger("costmap_2d"), "parameter callback: Thread Finished");

}

/* void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  map_update_thread_ = new std::thread(std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
} */

// TODO(bpwilcox): resolve dynamic reconfigure dependencies
/*
void Costmap2DROS::readFootprintFromConfig(const nav2_costmap_2d::Costmap2DConfig &new_config,
                                           const nav2_costmap_2d::Costmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("nav2_costmap_2d"),"Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(makeFootprintFromRadius(new_config.robot_radius));
  }
} */


void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

// TODO(bpwilcox): resolve dynamic reconfigure dependencies

void Costmap2DROS::movementCB()
{
  // don't allow configuration to happen while this check occurs
  // std::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  geometry_msgs::msg::PoseStamped new_pose;
  if (!getRobotPose(new_pose)) {
    RCLCPP_WARN(rclcpp::get_logger(
          "nav2_costmap_2d"), "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else {
    old_pose_ = new_pose;

    robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
          old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
            new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
        (tf2::Quaternion(old_pose_.pose.orientation.x,
          old_pose_.pose.orientation.y,
          old_pose_.pose.orientation.z,
          old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w)) < 1e-3);
  }
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0) {
    return;
  }

  rclcpp::Rate r(frequency);
  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);

    updateMap();

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "Map update time: %.9f", t_diff);
    if (publish_cycle_.nanoseconds() > 0 && layered_costmap_->isInitialized()) {
/*       unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      rclcpp::Time now = nh->now();
      //if (last_publish_.nanoseconds() + publish_cycle_.nanoseconds() < now.nanoseconds()) {
      if (last_publish_ + publish_cycle_ < now) {
        publisher_->publishCostmap();
        last_publish_ = now;
      } */
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time

    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
/*     if (r.period() > tf2::durationFromSec(1 / frequency)) {    
      RCLCPP_WARN(rclcpp::get_logger(
            "nav2_costmap_2d"),
          "Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
          frequency,
          r.period());
    } */
  }
}

void Costmap2DROS::updateMap()
{

  RCLCPP_INFO(rclcpp::get_logger("costmap_2d"), "Updating Map...");

  //auto clock = new rclcpp::Clock();
  rclcpp::Clock clock;
  if (!stop_updates_) {
    // get global pose
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      double x = pose.pose.position.x,
        y = pose.pose.position.y,
        yaw = tf2::getYaw(pose.pose.orientation);

      layered_costmap_->updateMap(x, y, yaw);
      geometry_msgs::msg::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = private_nh_->now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      footprint_pub_->publish(footprint);

      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin();
        plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (rclcpp::ok() && !initialized_) {
    r.sleep();
  }
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}


void Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector<std::shared_ptr<Layer> > * plugins = layered_costmap_->getPlugins();
  for (vector<std::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);

  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time();

  rclcpp::Time current_time = private_nh_->now();  // save time for checking tf delay later
  // get the global pose of the robot
  try {
    tf_.transform(robot_pose, global_pose, global_frame_);
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger(
          "nav2_costmap_2d"), "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout

  //TODO(bpwilcox): use toSec() function in more recent rclcpp branch
  if ((current_time - rclcpp::Time(global_pose.header.stamp)) > nav2_util::durationFromSeconds(transform_tolerance_))
  {
    RCLCPP_WARN(rclcpp::get_logger(
          "costmap_2d"),
        "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f, difference: %.4f",
        tf2::timeToSec(tf2_ros::fromMsg(current_time)),
        tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)), transform_tolerance_,
        tf2::timeToSec(tf2_ros::fromMsg(current_time)) -
          tf2::timeToSec(tf2_ros::fromMsg(global_pose.header.stamp)));

    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
const
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    return;
  }

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
      padded_footprint_, oriented_footprint);
}

}  // namespace nav2_costmap_2d
