// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_UTIL__SENSORS__LASER__LASER_HPP_
#define NAV2_UTIL__SENSORS__LASER__LASER_HPP_

#include <string>
#include "nav2_util/pf/pf.hpp"
#include "nav2_util/pf/pf_pdf.hpp"
#include "nav2_util/map/map.hpp"

namespace nav2_util
{

// Forward declarations
class LaserData;

class Laser
{
public:
  Laser(size_t max_beams, map_t * map);
  virtual ~Laser();
  virtual bool sensorUpdate(pf_t * pf, LaserData * data) = 0;
  void SetLaserPose(pf_vector_t & laser_pose);

protected:
  double z_hit_;
  double z_rand_;
  double sigma_hit_;

  void reallocTempData(int max_samples, int max_obs);
  map_t * map_;
  pf_vector_t laser_pose_;
  int max_beams_;
  int max_samples_;
  int max_obs_;
  double ** temp_obs_;
};

class LaserData
{
public:
  Laser * laser;
  LaserData() {ranges = NULL;}
  virtual ~LaserData() {delete[] ranges;}

public:
  int range_count;
  double range_max;
  double(*ranges)[2];
};


class BeamModel : public Laser
{
public:
  BeamModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams, map_t * map);
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  double z_short_;
  double z_max_;
  double lambda_short_;
  double chi_outlier_;
};

class LikelihoodFieldModel : public Laser
{
public:
  LikelihoodFieldModel(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    size_t max_beams, map_t * map);
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
};

class LikelihoodFieldModelProb : public Laser
{
public:
  LikelihoodFieldModelProb(
    double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
    bool do_beamskip, double beam_skip_distance,
    double beam_skip_threshold, double beam_skip_error_threshold,
    size_t max_beams, map_t * map);
  bool sensorUpdate(pf_t * pf, LaserData * data);

private:
  static double sensorFunction(LaserData * data, pf_sample_set_t * set);
  bool do_beamskip_;
  double beam_skip_distance_;
  double beam_skip_threshold_;
  double beam_skip_error_threshold_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__SENSORS__LASER__LASER_HPP_
