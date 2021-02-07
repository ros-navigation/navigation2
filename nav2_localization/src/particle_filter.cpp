#include <random>
#include <unordered_set>
#include "nav2_localization/particle_filter.hpp"
#include "tf2/utils.h"

namespace  nav2_localization
{
ParticleFilter::ParticleFilter(const int &initial_number_of_particles, const geometry_msgs::msg::TransformStamped &init_pose)
{
    initFilter(initial_number_of_particles, init_pose);
}

void ParticleFilter::initFilter(const int &initial_number_of_particles, const geometry_msgs::msg::TransformStamped &init_pose)
{
    particles_t_1_.clear();

    std::default_random_engine generator;
    std::normal_distribution<double> rand_distribution(0.0, 0.1);

    for(int i=0; i<initial_number_of_particles; i++)
    {
        double delta_x = rand_distribution(generator);
        double delta_y = rand_distribution(generator);

        geometry_msgs::msg::TransformStamped pose;
        pose.transform.translation.x = init_pose.transform.translation.x + delta_x;
        pose.transform.translation.y = init_pose.transform.translation.y + delta_y;
        pose.transform.rotation = init_pose.transform.rotation;
        particles_t_1_.push_back(Particle(pose, 1.0));  // arbitrarily assign a weight of 1.0
    }
}

void ParticleFilter::update(const geometry_msgs::msg::TransformStamped &prev_odom,
                            const geometry_msgs::msg::TransformStamped &curr_odom,
                            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
                            const SampleMotionModel::Ptr &motion_model,
                            const Matcher2d::Ptr &matcher)
{
    sum_of_weights_ = 0.0;
    std::vector<Particle> particles_t_bar = sample(prev_odom, curr_odom, scan, motion_model, matcher);

    for(auto p : particles_t_bar)
        p.weight_ /= sum_of_weights_;

    particles_t_ = resample(particles_t_bar);
    particles_t_1_ = particles_t_;

    updateStats();
}

std::vector<Particle> ParticleFilter::sample(const geometry_msgs::msg::TransformStamped &prev_odom,
                                             const geometry_msgs::msg::TransformStamped &curr_odom,
                                             const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
                                             const SampleMotionModel::Ptr &motion_model,
                                             const Matcher2d::Ptr &matcher)
{
    std::vector<Particle> particles_t_bar;
    for(auto p : particles_t_1_)
    {
        geometry_msgs::msg::TransformStamped most_likely_pose = 
                motion_model->getMostLikelyPose(prev_odom, curr_odom, p.pose_);
        double weight = matcher->getScanProbability(scan, most_likely_pose);
        particles_t_bar.push_back({most_likely_pose, weight});
        sum_of_weights_ += weight;
    }
    return particles_t_bar;
}

std::vector<Particle> ParticleFilter::resample(const std::vector<Particle> &particles_t_bar)
{
    std::vector<Particle> x_t;
    std::vector<double> weights;

    // extract all the weights into a vector
    for(auto particle : particles_t_bar)
        weights.push_back(particle.weight_);

    std::default_random_engine generator;
    std::discrete_distribution<int> weights_dist(weights.begin(), weights.end());

    std::unordered_set<int> indices_to_be_added;
    for(int i=0; i<particles_t_bar.size(); i++)
    {
        int drawn_particle_index = weights_dist(generator);
        x_t.push_back(particles_t_bar[drawn_particle_index]);
    }

    return x_t;
}

void ParticleFilter::updateStats()
{
    double mean_x=0.0, mean_y=0.0, mean_theta_cs=0.0, mean_theta_sn=0.0, total_weight=0.0;
    for(auto p : particles_t_)
    {
        total_weight += p.weight_;
        mean_x += p.pose_.transform.translation.x * p.weight_;
        mean_y += p.pose_.transform.translation.y * p.weight_;
        double angle = tf2::getYaw(p.pose_.transform.rotation);
        mean_theta_cs += cos(angle) * p.weight_;
        mean_theta_sn += sin(angle) * p.weight_;
    }

    mean_x /= total_weight;
    mean_y /= total_weight;
    double mean_theta = atan2(mean_theta_sn, mean_theta_cs);
    
    most_likely_pose_.transform.translation.x = mean_x;
    most_likely_pose_.transform.translation.y = mean_y;

    tf2::Quaternion theta_prime_quat;
    theta_prime_quat.setRPY(0.0, 0.0, mean_theta);
    most_likely_pose_.transform.rotation = tf2::toMsg(theta_prime_quat);
}

geometry_msgs::msg::TransformStamped ParticleFilter::getMostLikelyPose()
{
    return most_likely_pose_;
}

std::vector<Particle> ParticleFilter::getParticles()
{
    return particles_t_;
}

} // nav2_localization
