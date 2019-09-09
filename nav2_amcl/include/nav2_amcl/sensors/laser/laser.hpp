// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


#ifndef NAV2_AMCL__SENSORS__LASER__LASER_HPP_
#define NAV2_AMCL__SENSORS__LASER__LASER_HPP_

#include <string>
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/map/map.hpp"

namespace nav2_amcl
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

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__SENSORS__LASER__LASER_HPP_
