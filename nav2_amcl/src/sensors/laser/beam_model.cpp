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

BeamModel::BeamModel(
  double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
  double lambda_short, double chi_outlier, size_t max_beams, map_t * map)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  z_short_ = z_short;
  z_max_ = z_max;
  lambda_short_ = lambda_short;
  chi_outlier_ = chi_outlier;
}

// Determine the probability for the given pose
double
BeamModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  BeamModel * self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t * sample;
  pf_vector_t pose;

  self = reinterpret_cast<BeamModel *>(data->laser);

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams_ - 1);
    for (i = 0; i < data->range_count; i += step) {
      obs_range = data->ranges[i][0];

      // Check for NaN
      if (isnan(obs_range)) {
        continue;
      }

      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(
        self->map_, pose.v[0], pose.v[1],
        pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit_ * exp(-(z * z) / (2 * self->sigma_hit_ * self->sigma_hit_));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0) {
        pz += self->z_short_ * self->lambda_short_ * exp(-self->lambda_short_ * obs_range);
      }

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data->range_max) {
        pz += self->z_max_ * 1.0;
      }

      // Part 4: Random measurements
      if (obs_range < data->range_max) {
        pz += self->z_rand_ * 1.0 / data->range_max;
      }

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
BeamModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
