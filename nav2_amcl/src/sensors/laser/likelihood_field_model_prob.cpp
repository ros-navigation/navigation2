/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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


#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

LikelihoodFieldModelProb::LikelihoodFieldModelProb(
  double z_hit, double z_rand, double sigma_hit,
  double max_occ_dist, bool do_beamskip,
  double beam_skip_distance,
  double beam_skip_threshold,
  double beam_skip_error_threshold,
  size_t max_beams, map_t * map)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  do_beamskip_ = do_beamskip;
  beam_skip_distance_ = beam_skip_distance;
  beam_skip_threshold_ = beam_skip_threshold;
  beam_skip_error_threshold_ = beam_skip_error_threshold;
  map_update_cspace(map, max_occ_dist);
}

// Determine the probability for the given pose
double
LikelihoodFieldModelProb::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  LikelihoodFieldModelProb * self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t * sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = reinterpret_cast<LikelihoodFieldModelProb *>(data->laser);

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams_));

  // Step size must be at least 1
  if (step < 1) {
    step = 1;
  }

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_mult = 1.0 / data->range_max;

  double max_dist_prob = exp(-(self->map_->max_occ_dist * self->map_->max_occ_dist) / z_hit_denom);

  // Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  // prevents correct particles from getting down weighted because of unexpected obstacles
  // such as humans

  bool do_beamskip = self->do_beamskip_;
  double beam_skip_distance = self->beam_skip_distance_;
  double beam_skip_threshold = self->beam_skip_threshold_;

  // we only do beam skipping if the filter has converged
  if (do_beamskip && !set->converged) {
    do_beamskip = false;
  }

  // we need a count the no of particles for which the beam agreed with the map
  int * obs_count = new int[self->max_beams_]();

  // we also need a mask of which observations to integrate (to decide which beams to integrate to
  // all particles)
  bool * obs_mask = new bool[self->max_beams_]();

  int beam_ind = 0;

  // realloc indicates if we need to reallocate the temp data structure needed to do beamskipping
  bool realloc = false;

  if (do_beamskip) {
    if (self->max_obs_ < self->max_beams_) {
      realloc = true;
    }

    if (self->max_samples_ < set->sample_count) {
      realloc = true;
    }

    if (realloc) {
      self->reallocTempData(set->sample_count, self->max_beams_);
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples_, self->max_obs_);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max) {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range) {
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map_, hit.v[0]);
      mj = MAP_GYWY(self->map_, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if (!MAP_VALID(self->map_, mi, mj)) {
        pz += self->z_hit_ * max_dist_prob;
      } else {
        z = self->map_->cells[MAP_INDEX(self->map_, mi, mj)].occ_dist;
        if (z < beam_skip_distance) {
          obs_count[beam_ind] += 1;
        }
        pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += self->z_rand_ * z_rand_mult;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      // TODO(?): outlier rejection for short readings

      if (!do_beamskip) {
        log_p += log(pz);
      } else {
        self->temp_obs_[j][beam_ind] = pz;
      }
    }
    if (!do_beamskip) {
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }

  if (do_beamskip) {
    int skipped_beam_count = 0;
    for (beam_ind = 0; beam_ind < self->max_beams_; beam_ind++) {
      if ((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold) {
        obs_mask[beam_ind] = true;
      } else {
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    // we check if there is at least a critical number of beams that agreed with the map
    // otherwise it probably indicates that the filter converged to a wrong solution
    // if that's the case we integrate all the beams and hope the filter might converge to
    // the right solution
    bool error = false;

    if (skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold_)) {
      fprintf(
        stderr,
        "Over %f%% of the observations were not in the map - pf may have converged to wrong pose -"
        " integrating all observations\n",
        (100 * self->beam_skip_error_threshold_));
      error = true;
    }

    for (j = 0; j < set->sample_count; j++) {
      sample = set->samples + j;
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < self->max_beams_; beam_ind++) {
        if (error || obs_mask[beam_ind]) {
          log_p += log(self->temp_obs_[j][beam_ind]);
        }
      }

      sample->weight *= exp(log_p);

      total_weight += sample->weight;
    }
  }

  delete[] obs_count;
  delete[] obs_mask;
  return total_weight;
}

bool
LikelihoodFieldModelProb::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
