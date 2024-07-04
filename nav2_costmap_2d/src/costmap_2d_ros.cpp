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

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <utility>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/robot_utils.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{
Costmap2DROS::Costmap2DROS(const std::string & name, const bool & use_sim_time)
: Costmap2DROS(name, "/", name, use_sim_time) {}

Costmap2DROS::Costmap2DROS(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("costmap", "", options),
  name_("costmap"),
  default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},
  default_types_{
    "nav2_costmap_2d::StaticLayer",
    "nav2_costmap_2d::ObstacleLayer",
    "nav2_costmap_2d::InflationLayer"}
{
  declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
  is_lifecycle_follower_ = false;
  init();
}

Costmap2DROS::Costmap2DROS(
  const std::string & name,
  const std::string & parent_namespace,
  const std::string & local_namespace,
  const bool & use_sim_time)
: nav2_util::LifecycleNode(name, "",
    // NodeOption arguments take precedence over the ones provided on the command line
    // use this to make sure the node is placed on the provided namespace
    // TODO(orduno) Pass a sub-node instead of creating a new node for better handling
    //              of the namespaces
    rclcpp::NodeOptions().arguments({
    "--ros-args", "-r", std::string("__ns:=") +
    nav2_util::add_namespaces(parent_namespace, local_namespace),
    "--ros-args", "-r", name + ":" + std::string("__node:=") + name,
    "--ros-args", "-p", "use_sim_time:=" + std::string(use_sim_time ? "true" : "false"),
  })),
  name_(name),
  parent_namespace_(parent_namespace),
  default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},
  default_types_{
    "nav2_costmap_2d::StaticLayer",
    "nav2_costmap_2d::ObstacleLayer",
    "nav2_costmap_2d::InflationLayer"}
{
  declare_parameter(
    "map_topic", rclcpp::ParameterValue(
      (parent_namespace_ == "/" ? "/" : parent_namespace_ + "/") + std::string("map")));
  init();
}

void Costmap2DROS::init()
{
  RCLCPP_INFO(get_logger(), "Creating Costmap");

  declare_parameter("always_send_full_costmap", rclcpp::ParameterValue(false));
  declare_parameter("footprint_padding", rclcpp::ParameterValue(0.01f));
  declare_parameter("footprint", rclcpp::ParameterValue(std::string("[]")));
  declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("height", rclcpp::ParameterValue(5));
  declare_parameter("width", rclcpp::ParameterValue(5));
  declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
  declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declare_parameter("origin_x", rclcpp::ParameterValue(0.0));
  declare_parameter("origin_y", rclcpp::ParameterValue(0.0));
  declare_parameter("plugins", rclcpp::ParameterValue(default_plugins_));
  declare_parameter("filters", rclcpp::ParameterValue(std::vector<std::string>()));
  declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0));
  declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter("robot_radius", rclcpp::ParameterValue(0.1));
  declare_parameter("rolling_window", rclcpp::ParameterValue(false));
  declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declare_parameter("initial_transform_timeout", rclcpp::ParameterValue(60.0));
  declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  declare_parameter("unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
  declare_parameter("update_frequency", rclcpp::ParameterValue(5.0));
  declare_parameter("use_maximum", rclcpp::ParameterValue(false));
}

Costmap2DROS::~Costmap2DROS()
{
}

nav2_util::CallbackReturn
Costmap2DROS::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  try {
    getParameters();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to configure costmap! %s.", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // Create the costmap itself
  layered_costmap_ = std::make_unique<LayeredCostmap>(
    global_frame_, rolling_window_, track_unknown_space_);

  if (!layered_costmap_->isSizeLocked()) {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),
      (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
  }

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Then load and add the plug-ins to the costmap
  for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Using plugin \"%s\"", plugin_names_[i].c_str());

    std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);

    // lock the costmap because no update is allowed until the plugin is initialized
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

    layered_costmap_->addPlugin(plugin);

    // TODO(mjeronimo): instead of get(), use a shared ptr
    plugin->initialize(
      layered_costmap_.get(), plugin_names_[i], tf_buffer_.get(),
      shared_from_this(), callback_group_);

    lock.unlock();

    RCLCPP_INFO(get_logger(), "Initialized plugin \"%s\"", plugin_names_[i].c_str());
  }
  // and costmap filters as well
  for (unsigned int i = 0; i < filter_names_.size(); ++i) {
    RCLCPP_INFO(get_logger(), "Using costmap filter \"%s\"", filter_names_[i].c_str());

    std::shared_ptr<Layer> filter = plugin_loader_.createSharedInstance(filter_types_[i]);

    // lock the costmap because no update is allowed until the filter is initialized
    std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

    layered_costmap_->addFilter(filter);

    filter->initialize(
      layered_costmap_.get(), filter_names_[i], tf_buffer_.get(),
      shared_from_this(), callback_group_);

    lock.unlock();

    RCLCPP_INFO(get_logger(), "Initialized costmap filter \"%s\"", filter_names_[i].c_str());
  }

  // Create the publishers and subscribers
  footprint_sub_ = create_subscription<geometry_msgs::msg::Polygon>(
    "footprint",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&Costmap2DROS::setRobotFootprintPolygon, this, std::placeholders::_1));

  footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "published_footprint", rclcpp::SystemDefaultsQoS());

  costmap_publisher_ = std::make_unique<Costmap2DPublisher>(
    shared_from_this(),
    layered_costmap_->getCostmap(), global_frame_,
    "costmap", always_send_full_costmap_);

  auto layers = layered_costmap_->getPlugins();

  for (auto & layer : *layers) {
    auto costmap_layer = std::dynamic_pointer_cast<CostmapLayer>(layer);
    if (costmap_layer != nullptr) {
      layer_publishers_.emplace_back(
        std::make_unique<Costmap2DPublisher>(
          shared_from_this(),
          costmap_layer.get(), global_frame_,
          layer->getName(), always_send_full_costmap_)
      );
    }
  }

  // Set the footprint
  if (use_radius_) {
    setRobotFootprint(makeFootprintFromRadius(robot_radius_));
  } else {
    std::vector<geometry_msgs::msg::Point> new_footprint;
    makeFootprintFromString(footprint_, new_footprint);
    setRobotFootprint(new_footprint);
  }

  // Add cleaning service
  clear_costmap_service_ = std::make_unique<ClearCostmapService>(shared_from_this(), *this);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // First, make sure that the transform between the robot base frame
  // and the global frame is available

  std::string tf_error;

  RCLCPP_INFO(get_logger(), "Checking transform");
  rclcpp::Rate r(2);
  const auto initial_transform_timeout = rclcpp::Duration::from_seconds(
    initial_transform_timeout_);
  const auto initial_transform_timeout_point = now() + initial_transform_timeout;
  while (rclcpp::ok() &&
    !tf_buffer_->canTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());

    // Check timeout
    if (now() > initial_transform_timeout_point) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to activate %s because "
        "transform from %s to %s did not become available before timeout",
        get_name(), robot_base_frame_.c_str(), global_frame_.c_str());

      return nav2_util::CallbackReturn::FAILURE;
    }

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }

  // Activate publishers
  footprint_pub_->on_activate();
  costmap_publisher_->on_activate();

  for (auto & layer_pub : layer_publishers_) {
    layer_pub->on_activate();
  }

  // Create a thread to handle updating the map
  stopped_ = true;  // to active plugins
  stop_updates_ = false;
  map_update_thread_shutdown_ = false;
  map_update_thread_ = std::make_unique<std::thread>(
    std::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency_));

  start();

  // Add callback for dynamic parameters
  dyn_params_handler = this->add_on_set_parameters_callback(
    std::bind(&Costmap2DROS::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  dyn_params_handler.reset();

  stop();

  // Map thread stuff
  map_update_thread_shutdown_ = true;

  if (map_update_thread_->joinable()) {
    map_update_thread_->join();
  }

  footprint_pub_->on_deactivate();
  costmap_publisher_->on_deactivate();

  for (auto & layer_pub : layer_publishers_) {
    layer_pub->on_deactivate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  costmap_publisher_.reset();
  clear_costmap_service_.reset();

  layer_publishers_.clear();

  layered_costmap_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  footprint_sub_.reset();
  footprint_pub_.reset();


  executor_thread_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Costmap2DROS::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
Costmap2DROS::getParameters()
{
  RCLCPP_DEBUG(get_logger(), " getParameters");

  // Get all of the required parameters
  get_parameter("always_send_full_costmap", always_send_full_costmap_);
  get_parameter("footprint", footprint_);
  get_parameter("footprint_padding", footprint_padding_);
  get_parameter("global_frame", global_frame_);
  get_parameter("height", map_height_meters_);
  get_parameter("origin_x", origin_x_);
  get_parameter("origin_y", origin_y_);
  get_parameter("publish_frequency", map_publish_frequency_);
  get_parameter("resolution", resolution_);
  get_parameter("robot_base_frame", robot_base_frame_);
  get_parameter("robot_radius", robot_radius_);
  get_parameter("rolling_window", rolling_window_);
  get_parameter("track_unknown_space", track_unknown_space_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("initial_transform_timeout", initial_transform_timeout_);
  get_parameter("update_frequency", map_update_frequency_);
  get_parameter("width", map_width_meters_);
  get_parameter("plugins", plugin_names_);
  get_parameter("filters", filter_names_);

  auto node = shared_from_this();

  if (plugin_names_ == default_plugins_) {
    for (size_t i = 0; i < default_plugins_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_plugins_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }
  plugin_types_.resize(plugin_names_.size());
  filter_types_.resize(filter_names_.size());

  // 1. All plugins must have 'plugin' param defined in their namespace to define the plugin type
  for (size_t i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = nav2_util::get_plugin_type_param(node, plugin_names_[i]);
  }
  for (size_t i = 0; i < filter_names_.size(); ++i) {
    filter_types_[i] = nav2_util::get_plugin_type_param(node, filter_names_[i]);
  }

  // 2. The map publish frequency cannot be 0 (to avoid a divde-by-zero)
  if (map_publish_frequency_ > 0) {
    publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
  } else {
    publish_cycle_ = rclcpp::Duration(-1s);
  }

  // 3. If the footprint has been specified, it must be in the correct format
  use_radius_ = true;

  if (footprint_ != "" && footprint_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(footprint_, new_footprint)) {
      // The specified footprint is valid, so we'll use that instead of the radius
      use_radius_ = false;
    } else {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        get_logger(), "The footprint parameter is invalid: \"%s\", using radius (%lf) instead",
        footprint_.c_str(), robot_radius_);
    }
  }

  // 4. The width and height of map cannot be negative or 0 (to avoid abnoram memory usage)
  if (map_width_meters_ <= 0) {
    RCLCPP_ERROR(
      get_logger(), "You try to set width of map to be negative or zero,"
      " this isn't allowed, please give a positive value.");
  }
  if (map_height_meters_ <= 0) {
    RCLCPP_ERROR(
      get_logger(), "You try to set height of map to be negative or zero,"
      " this isn't allowed, please give a positive value.");
  }
}

void
Costmap2DROS::setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);
  layered_costmap_->setFootprint(padded_footprint_);
}

void
Costmap2DROS::setRobotFootprintPolygon(
  const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  setRobotFootprint(toPointVector(footprint));
}

void
Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    return;
  }

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw,
    padded_footprint_, oriented_footprint);
}

void
Costmap2DROS::mapUpdateLoop(double frequency)
{
  RCLCPP_DEBUG(get_logger(), "mapUpdateLoop frequency: %lf", frequency);

  // the user might not want to run the loop every cycle
  if (frequency == 0.0) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Entering loop");

  rclcpp::WallRate r(frequency);    // 200ms by default

  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    nav2_util::ExecutionTimer timer;

    // Execute after start() will complete plugins activation
    if (!stopped_) {
      // Lock while modifying layered costmap and publishing values
      std::scoped_lock<std::mutex> lock(_dynamic_parameter_mutex);

      // Measure the execution time of the updateMap method
      timer.start();
      updateMap();
      timer.end();

      RCLCPP_DEBUG(get_logger(), "Map update time: %.9f", timer.elapsed_time_in_seconds());
      if (publish_cycle_ > rclcpp::Duration(0s) && layered_costmap_->isInitialized()) {
        unsigned int x0, y0, xn, yn;
        layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
        costmap_publisher_->updateBounds(x0, xn, y0, yn);

        for (auto & layer_pub : layer_publishers_) {
          layer_pub->updateBounds(x0, xn, y0, yn);
        }

        auto current_time = now();
        if ((last_publish_ + publish_cycle_ < current_time) ||  // publish_cycle_ is due
          (current_time <
          last_publish_))      // time has moved backwards, probably due to a switch to sim_time // NOLINT
        {
          RCLCPP_DEBUG(get_logger(), "Publish costmap at %s", name_.c_str());
          costmap_publisher_->publishCostmap();

          for (auto & layer_pub : layer_publishers_) {
            layer_pub->publishCostmap();
          }

          last_publish_ = current_time;
        }
      }
    }

    // Make sure to sleep for the remainder of our cycle time
    r.sleep();

#if 0
    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
    if (r.period() > tf2::durationFromSec(1 / frequency)) {
      RCLCPP_WARN(
        get_logger(),
        "Costmap2DROS: Map update loop missed its desired rate of %.4fHz... "
        "the loop actually took %.4f seconds", frequency, r.period());
    }
#endif
  }
}

void
Costmap2DROS::updateMap()
{
  RCLCPP_DEBUG(get_logger(), "Updating map...");

  if (!stop_updates_) {
    // get global pose
    geometry_msgs::msg::PoseStamped pose;
    if (getRobotPose(pose)) {
      const double & x = pose.pose.position.x;
      const double & y = pose.pose.position.y;
      const double yaw = tf2::getYaw(pose.pose.orientation);
      layered_costmap_->updateMap(x, y, yaw);

      auto footprint = std::make_unique<geometry_msgs::msg::PolygonStamped>();
      footprint->header = pose.header;
      transformFootprint(x, y, yaw, padded_footprint_, *footprint);

      RCLCPP_DEBUG(get_logger(), "Publishing footprint");
      footprint_pub_->publish(std::move(footprint));
      initialized_ = true;
    }
  }
}

void
Costmap2DROS::start()
{
  RCLCPP_INFO(get_logger(), "start");
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

  // check if we're stopped or just paused
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
      plugin != plugins->end();
      ++plugin)
    {
      (*plugin)->activate();
    }
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
      filter != filters->end();
      ++filter)
    {
      (*filter)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(20.0);
  while (rclcpp::ok() && !initialized_) {
    RCLCPP_DEBUG(get_logger(), "Sleeping, waiting for initialized_");
    r.sleep();
  }
}

void
Costmap2DROS::stop()
{
  stop_updates_ = true;

  // layered_costmap_ is set only if on_configure has been called
  if (layered_costmap_) {
    std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
    std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

    // unsubscribe from topics
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
      plugin != plugins->end(); ++plugin)
    {
      (*plugin)->deactivate();
    }
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
      filter != filters->end(); ++filter)
    {
      (*filter)->deactivate();
    }
  }
  initialized_ = false;
  stopped_ = true;
}

void
Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void
Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}

void
Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

  // Reset each of the plugins
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();
  for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
    plugin != plugins->end(); ++plugin)
  {
    (*plugin)->reset();
  }
  for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
    filter != filters->end(); ++filter)
  {
    (*filter)->reset();
  }
}

bool
Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
{
  return nav2_util::getCurrentPose(
    global_pose, *tf_buffer_,
    global_frame_, robot_base_frame_, transform_tolerance_);
}

bool
Costmap2DROS::transformPoseToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose)
{
  if (input_pose.header.frame_id == global_frame_) {
    transformed_pose = input_pose;
    return true;
  } else {
    return nav2_util::transformPoseInTargetFrame(
      input_pose, transformed_pose, *tf_buffer_,
      global_frame_, transform_tolerance_);
  }
}

rcl_interfaces::msg::SetParametersResult
Costmap2DROS::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  bool resize_map = false;
  std::lock_guard<std::mutex> lock_reinit(_dynamic_parameter_mutex);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "robot_radius") {
        robot_radius_ = parameter.as_double();
        // Set the footprint
        if (use_radius_) {
          setRobotFootprint(makeFootprintFromRadius(robot_radius_));
        }
      } else if (name == "footprint_padding") {
        footprint_padding_ = parameter.as_double();
        padded_footprint_ = unpadded_footprint_;
        padFootprint(padded_footprint_, footprint_padding_);
        layered_costmap_->setFootprint(padded_footprint_);
      } else if (name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      } else if (name == "publish_frequency") {
        map_publish_frequency_ = parameter.as_double();
        if (map_publish_frequency_ > 0) {
          publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
        } else {
          publish_cycle_ = rclcpp::Duration(-1s);
        }
      } else if (name == "resolution") {
        resize_map = true;
        resolution_ = parameter.as_double();
      } else if (name == "origin_x") {
        resize_map = true;
        origin_x_ = parameter.as_double();
      } else if (name == "origin_y") {
        resize_map = true;
        origin_y_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "width") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_width_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(), "You try to set width of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      } else if (name == "height") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_height_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(), "You try to set height of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "footprint") {
        footprint_ = parameter.as_string();
        std::vector<geometry_msgs::msg::Point> new_footprint;
        if (makeFootprintFromString(footprint_, new_footprint)) {
          setRobotFootprint(new_footprint);
        }
      } else if (name == "robot_base_frame") {
        // First, make sure that the transform between the robot base frame
        // and the global frame is available
        std::string tf_error;
        RCLCPP_INFO(get_logger(), "Checking transform");
        if (!tf_buffer_->canTransform(
            global_frame_, parameter.as_string(), tf2::TimePointZero,
            tf2::durationFromSec(1.0), &tf_error))
        {
          RCLCPP_WARN(
            get_logger(), "Timed out waiting for transform from %s to %s"
            " to become available, tf error: %s",
            parameter.as_string().c_str(), global_frame_.c_str(), tf_error.c_str());
          RCLCPP_WARN(
            get_logger(), "Rejecting robot_base_frame change to %s , leaving it to its original"
            " value of %s", parameter.as_string().c_str(), robot_base_frame_.c_str());
          result.successful = false;
          return result;
        }
        robot_base_frame_ = parameter.as_string();
      }
    }
  }

  if (resize_map && !layered_costmap_->isSizeLocked()) {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),
      (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
    updateMap();
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_costmap_2d
