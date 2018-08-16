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

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

//#include "ros/assert.h"

// roscpp
#include "rclcpp/rclcpp.hpp"

// Messages that I need
//#include "sensor_msgs/LaserScan.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include "geometry_msgs/PoseArray.h"
//#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "nav_msgs/GetMap.h"
//#include "nav_msgs/SetMap.h"
//#include "std_srvs/Empty.h"

// For transform support
//#include "tf2/LinearMath/Transform.h"
//#include "tf2/convert.h"
//#include "tf2/utils.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "tf2_ros/buffer.h"
//#include "tf2_ros/message_filter.h"
//#include "tf2_ros/transform_broadcaster.h"
//#include "tf2_ros/transform_listener.h"
//#include "message_filters/subscriber.h"

// Dynamic_reconfigure
//#include "dynamic_reconfigure/server.h"
//#include "amcl/AMCLConfig.h"

// Allows AMCL to run from bag file
#if 0
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#endif

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

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

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

static const std::string scan_topic_ = "scan";

/* This function is only useful to have the whole code work
 * with old rosbags that have trailing slashes for their frames
 */
inline
std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if ( ( !in.empty() ) && (in[0] == '/') )
    out.erase(0,1);
  return out;
}

#if 0
class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     */
    void runFromBag(const std::string &in_bag_fn);

    int process();
    void savePoseToServer();

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    bool sent_first_transform_;

    tf2::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool setMapCallback(nav_msgs::SetMap::Request& req,
                        nav_msgs::SetMap::Response& res);

    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    geometry_msgs::PoseStamped latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf2_ros::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    ros::Subscriber initial_pose_sub_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(geometry_msgs::PoseStamped& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);
};

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}


AmclNode::AmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
	      private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);
  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);
  
  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);

  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob"){
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  private_nh_.param("tf_broadcast", tf_broadcast_, true);

  transform_tolerance_.fromSec(tmp_tol);

  {
    double bag_scan_period;
    private_nh_.param("bag_scan_period", bag_scan_period, -1.0);
    bag_scan_period_.fromSec(bag_scan_period);
  }

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  updatePoseFromServer();

  cloud_pub_interval.fromSec(1.0);
  tfb_.reset(new tf2_ros::TransformBroadcaster());
  tf_.reset(new tf2_ros::Buffer());
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
					 &AmclNode::globalLocalizationCallback,
                                         this);
  nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &AmclNode::nomotionUpdateCallback, this);
  set_map_srv_= nh_.advertiseService("set_map", &AmclNode::setMapCallback, this);

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ = 
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             nh_);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }
  m_force_update = false;

  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_, 
                                       boost::bind(&AmclNode::checkLaserReceived, this, _1));
}

void AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0/config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  max_beams_ = config.laser_max_beams;
  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if(config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(config.laser_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;

  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;

  do_beamskip_= config.do_beamskip; 
  beam_skip_distance_ = config.beam_skip_distance; 
  beam_skip_threshold_ = config.beam_skip_threshold; 
  
  if( pf_ != NULL )
  {
    pf_free( pf_ );
    pf_ = NULL;
  }	
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_err_ = config.kld_err; 
  pf_z_ = config.kld_z; 
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf2::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					laser_likelihood_max_dist_, 
					do_beamskip_, beam_skip_distance_, 
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  odom_frame_id_ = stripSlash(config.odom_frame_id);
  base_frame_id_ = stripSlash(config.base_frame_id);
  global_frame_id_ = stripSlash(config.global_frame_id);

  delete laser_scan_filter_;
  laser_scan_filter_ = 
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             nh_);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}


void AmclNode::runFromBag(const std::string &in_bag_fn)
{
  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  std::string scan_topic_name = "base_scan"; // TODO determine what topic this actually is from ROS
  topics.push_back(scan_topic_name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);

  // Sleep for a second to let all subscribers connect
  ros::WallDuration(1.0).sleep();

  ros::WallTime start(ros::WallTime::now());

  // Wait for map
  while (ros::ok())
  {
    {
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if (map_)
      {
        ROS_INFO("Map is ready");
        break;
      }
    }
    ROS_INFO("Waiting for map...");
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
  }

  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    if (!ros::ok())
    {
      break;
    }

    // Process any ros messages or callbacks at this point
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());

    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
      tf_pub.publish(msg);
      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
      {
        tf_->setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL)
    {
      laser_pub.publish(msg);
      laser_scan_filter_->add(base_scan);
      if (bag_scan_period_ > ros::WallDuration(0))
      {
        bag_scan_period_.sleep();
      }
      continue;
    }

    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
  }

  bag.close();

  double runtime = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::Quaternion & q(last_published_pose.pose.pose.orientation);
  double yaw, pitch, roll;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f",
            last_published_pose.pose.pose.position.x,
            last_published_pose.pose.pose.position.y,
            yaw, last_published_pose.header.stamp.toSec()
            );

  ros::shutdown();
}


void AmclNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf2::Transform odom_pose_tf2;
  tf2::convert(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

  double yaw = tf2::getYaw(map_pose.getRotation());

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx", 
                                  last_published_pose.pose.covariance[6*0+0]);
  private_nh_.setParam("initial_cov_yy", 
                                  last_published_pose.pose.covariance[6*1+1]);
  private_nh_.setParam("initial_cov_aa", 
                                  last_published_pose.pose.covariance[6*5+5]);
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
  if(!std::isnan(tmp_pos))
    init_pose_[0] = tmp_pos;
  else 
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
  if(!std::isnan(tmp_pos))
    init_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
  if(!std::isnan(tmp_pos))
    init_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
  if(!std::isnan(tmp_pos))
    init_cov_[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
  if(!std::isnan(tmp_pos))
    init_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
  if(!std::isnan(tmp_pos))
    init_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");	
}

void 
AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

void
AmclNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);
  
  if(msg.header.frame_id != global_frame_id_)
    ROS_WARN("Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could cause issues with reading published topics",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  updatePoseFromServer();
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					laser_likelihood_max_dist_, 
					do_beamskip_, beam_skip_distance_, 
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();

}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  delete dsrv_;
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
  // TODO: delete everything allocated in constructor
}

bool
AmclNode::getOdomPose(geometry_msgs::PoseStamped& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  geometry_msgs::PoseStamped ident;
  ident.header.frame_id = stripSlash(f);
  ident.header.stamp = t;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try
  {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  }
  catch(tf2::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);
  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool 
AmclNode::nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
	m_force_update = true;
	//ROS_INFO("Requesting no-motion update");
	return true;
}

bool
AmclNode::setMapCallback(nav_msgs::SetMap::Request& req,
                         nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}

void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan_frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = ros::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    geometry_msgs::PoseStamped laser_pose;
    try
    {
      this->tf_->transform(ident, laser_pose, base_frame_id_);
    }
    catch(tf2::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }


  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;

    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    geometry_msgs::QuaternionStamped min_q, inc_q;
    min_q.header.stamp = laser_scan->header.stamp;
    min_q.header.frame_id = stripSlash(laser_scan->header.frame_id);
    tf2::convert(q, min_q.quaternion);

    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    inc_q.header = min_q.header;
    tf2::convert(q, inc_q.quaternion);
    try
    {
      tf_->transform(min_q, min_q, base_frame_id_);
      tf_->transform(inc_q, inc_q, base_frame_id_);
    }
    catch(tf2::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf2::getYaw(min_q.quaternion);
    double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min +
              (i * angle_increment);
    }

    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    if (!m_force_update)
    {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
        cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
        cloud_msg.poses[i].position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, set->samples[i].pose.v[2]);
        tf2::convert(q, cloud_msg.poses[i].orientation);
      }
      particlecloud_pub_.publish(cloud_msg);
    }
  }

  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf2::Quaternion q;
      q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
      tf2::convert(q, p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      pose_pub_.publish(p);
      last_published_pose = p;

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      geometry_msgs::PoseStamped odom_to_map;
      try
      {
        tf2::Quaternion q;
        q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
        tf2::Transform tmp_tf(q, tf2::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                              hyps[max_weight_hyp].pf_pose_mean.v[1],
                                              0.0));

        geometry_msgs::PoseStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = base_frame_id_;
        tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
        tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

        this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
      }
      catch(tf2::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      tf2::convert(odom_to_map.pose, latest_tf_);
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (laser_scan->header.stamp +
                                          transform_tolerance_);
        geometry_msgs::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = global_frame_id_;
        tmp_tf_stamped.header.stamp = transform_expiration;
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
        this->tfb_->sendTransform(tmp_tf_stamped);
        sent_first_transform_ = true;
      }
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      geometry_msgs::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = transform_expiration;
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

}

void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(stripSlash(msg.header.frame_id) != global_frame_id_)
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             stripSlash(msg.header.frame_id).c_str(),
             global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::TransformStamped tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tx_odom = tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                                   base_frame_id_, ros::Time::now(),
                                   odom_frame_id_, ros::Duration(0.5));
  }
  catch(tf2::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tf2::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Transform tx_odom_tf2;
  tf2::convert(tx_odom.transform, tx_odom_tf2);
  tf2::Transform pose_old, pose_new;
  tf2::convert(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
            tf2::getYaw(pose_new.getRotation()));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}
#endif

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // , "amcl");

#if 0
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }
  else if ((argc == 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    amcl_node_ptr->runFromBag(argv[2]);
  }

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();
#endif

  rclcpp::shutdown();
  return 0;
}
