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

LikelihoodFieldModel::LikelihoodFieldModel(
  double z_hit, double z_rand, double sigma_hit,
  double max_occ_dist, size_t max_beams, map_t * map)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  map_update_cspace(map, max_occ_dist);
}

double
LikelihoodFieldModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  LikelihoodFieldModel * self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t * sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = reinterpret_cast<LikelihoodFieldModel *>(data->laser);

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_mult = 1.0 / data->range_max;

  step = (data->range_count - 1) / (self->max_beams_ - 1);

  // Step size must be at least 1
  if (step < 1) {
    step = 1;
  }

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    p = 1.0;

    for (i = 0; i < data->range_count; i += step) {
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
        z = self->map_->max_occ_dist;
      } else {
        z = self->map_->cells[MAP_INDEX(self->map_, mi, mj)].occ_dist;
      }
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand_ * z_rand_mult;

      // TODO(?): outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return total_weight;
}


bool
LikelihoodFieldModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
