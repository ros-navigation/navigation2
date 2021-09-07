#include <nav2_amcl/motion_model/motion_model.hpp>
#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav2_amcl/angleutils.hpp"

namespace nav2_amcl
{

class OmniMotionModel : public nav2_amcl::MotionModel
{
  public:
    void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) override
    {

  // Compute the new sample poses
  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  double delta_trans, delta_rot, delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

  delta_trans = sqrt(
    delta.v[0] * delta.v[0] +
    delta.v[1] * delta.v[1]);
  delta_rot = delta.v[2];

  // Precompute a couple of things
  double trans_hat_stddev = sqrt(
    alpha3_ * (delta_trans * delta_trans) +
    alpha4_ * (delta_rot * delta_rot) );
  double rot_hat_stddev = sqrt(
    alpha1_ * (delta_rot * delta_rot) +
    alpha2_ * (delta_trans * delta_trans) );
  double strafe_hat_stddev = sqrt(
    alpha4_ * (delta_rot * delta_rot) +
    alpha5_ * (delta_trans * delta_trans) );

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    delta_bearing = nav2_amcl::angleutils::angle_diff(
      atan2(delta.v[1], delta.v[0]),
      old_pose.v[2]) + sample->pose.v[2];
    double cs_bearing = cos(delta_bearing);
    double sn_bearing = sin(delta_bearing);

    // Sample pose differences
    delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
    delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
    delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
    // Apply sampled update to particle pose
    sample->pose.v[0] += (delta_trans_hat * cs_bearing +
      delta_strafe_hat * sn_bearing);
    sample->pose.v[1] += (delta_trans_hat * sn_bearing -
      delta_strafe_hat * cs_bearing);
    sample->pose.v[2] += delta_rot_hat;
  }
}
};

class DifferentialMotionModel : public nav2_amcl::MotionModel
{
  public:
     void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) override
    {
   RCLCPP_WARN(rclcpp::get_logger("my_logger"), "my_message_entered_omni_model");
  // Compute the new sample poses
  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  // Implement sample_motion_odometry (Prob Rob p 136)
  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;

  // Avoid computing a bearing from two poses that are extremely near each
  // other (happens on in-place rotation).
  if (sqrt(
      delta.v[1] * delta.v[1] +
      delta.v[0] * delta.v[0]) < 0.01)
  {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = nav2_amcl::angleutils::angle_diff(
      atan2(delta.v[1], delta.v[0]),
      old_pose.v[2]);
  }
  delta_trans = sqrt(
    delta.v[0] * delta.v[0] +
    delta.v[1] * delta.v[1]);
  delta_rot2 = nav2_amcl::angleutils::angle_diff(delta.v[2], delta_rot1);

  // We want to treat backward and forward motion symmetrically for the
  // noise model to be applied below.  The standard model seems to assume
  // forward motion.
  delta_rot1_noise = std::min(
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, 0.0)),
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, M_PI)));
  delta_rot2_noise = std::min(
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, 0.0)),
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, M_PI)));

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    // Sample pose differences
    delta_rot1_hat = nav2_amcl::angleutils::angle_diff(
      delta_rot1,
      pf_ran_gaussian(
        sqrt(
          alpha1_ * delta_rot1_noise * delta_rot1_noise +
          alpha2_ * delta_trans * delta_trans)));
    delta_trans_hat = delta_trans -
      pf_ran_gaussian(
      sqrt(
        alpha3_ * delta_trans * delta_trans +
        alpha4_ * delta_rot1_noise * delta_rot1_noise +
        alpha4_ * delta_rot2_noise * delta_rot2_noise));
    delta_rot2_hat = nav2_amcl::angleutils::angle_diff(
      delta_rot2,
      pf_ran_gaussian(
        sqrt(
          alpha1_ * delta_rot2_noise * delta_rot2_noise +
          alpha2_ * delta_trans * delta_trans)));

    // Apply sampled update to particle pose
    sample->pose.v[0] += delta_trans_hat *
      cos(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[1] += delta_trans_hat *
      sin(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
  }
}

};

class DifferentialMotionModelNew : public nav2_amcl::MotionModel
{
  public:
     void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) override
    {
    
    
  // Compute the new sample poses
  pf_sample_set_t * set;

  set = pf->sets + pf->current_set;
  pf_vector_t old_pose = pf_vector_sub(pose, delta);

  // Implement sample_motion_odometry (Prob Rob p 136)
  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;

  // Avoid computing a bearing from two poses that are extremely near each
  // other (happens on in-place rotation).
  if (sqrt(
      delta.v[1] * delta.v[1] +
      delta.v[0] * delta.v[0]) < 0.01)
  {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = nav2_amcl::angleutils::angle_diff(
      atan2(delta.v[1], delta.v[0]),
      old_pose.v[2]);
  }
  delta_trans = sqrt(
    delta.v[0] * delta.v[0] +
    delta.v[1] * delta.v[1]);
  delta_rot2 = nav2_amcl::angleutils::angle_diff(delta.v[2], delta_rot1);

  // We want to treat backward and forward motion symmetrically for the
  // noise model to be applied below.  The standard model seems to assume
  // forward motion.
  delta_rot1_noise = std::min(
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, 0.0)),
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, M_PI)));
  delta_rot2_noise = std::min(
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, 0.0)),
    fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, M_PI)));

  for (int i = 0; i < set->sample_count; i++) {
    pf_sample_t * sample = set->samples + i;

    // Sample pose differences
    delta_rot1_hat = nav2_amcl::angleutils::angle_diff(
      delta_rot1,
      pf_ran_gaussian(
        sqrt(
          alpha1_ * delta_rot1_noise * delta_rot1_noise +
          alpha2_ * delta_trans * delta_trans)));
    delta_trans_hat = delta_trans -
      pf_ran_gaussian(
      sqrt(
        alpha3_ * delta_trans * delta_trans +
        alpha4_ * delta_rot1_noise * delta_rot1_noise +
        alpha4_ * delta_rot2_noise * delta_rot2_noise));
    delta_rot2_hat = nav2_amcl::angleutils::angle_diff(
      delta_rot2,
      pf_ran_gaussian(
        sqrt(
          alpha1_ * delta_rot2_noise * delta_rot2_noise +
          alpha2_ * delta_trans * delta_trans)));

    // Apply sampled update to particle pose
    sample->pose.v[0] += delta_trans_hat *
      cos(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[1] += delta_trans_hat *
      sin(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
   }
  }
 };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_amcl::OmniMotionModel, nav2_amcl::MotionModel)
PLUGINLIB_EXPORT_CLASS(nav2_amcl::DifferentialMotionModel, nav2_amcl::MotionModel)
PLUGINLIB_EXPORT_CLASS(nav2_amcl::DifferentialMotionModelNew, nav2_amcl::MotionModel)
