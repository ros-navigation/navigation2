/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: Odometry sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_odom.h 4135 2007-08-23 19:58:48Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef NAV2_UTIL__SENSORS__ODOM_H_
#define NAV2_UTIL__SENSORS__ODOM_H_

#include "nav2_util/sensors/sensor.h"
#include "nav2_util/pf/pf_pdf.h"

namespace amcl
{

typedef enum
{
  ODOM_MODEL_DIFF,
  ODOM_MODEL_OMNI
} odom_model_t;

// Odometric sensor data
class OdomData : public SensorData
{
  // Odometric pose

public:
  pf_vector_t pose;

  // Change in odometric pose

public:
  pf_vector_t delta;
};


// Odometric sensor model
class Odom : public Sensor
{
  // Default constructor

public:
  Odom();

public:
  void SetModelDiff(
    double alpha1,
    double alpha2,
    double alpha3,
    double alpha4);

public:
  void SetModelOmni(
    double alpha1,
    double alpha2,
    double alpha3,
    double alpha4,
    double alpha5);

public:
  void SetModel(
    odom_model_t type,
    double alpha1,
    double alpha2,
    double alpha3,
    double alpha4,
    double alpha5 = 0);

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.

public:
  virtual bool UpdateAction(pf_t * pf, SensorData * data);

  // Current data timestamp

private:
  double time;

  // Model type

private:
  odom_model_t model_type;

  // Drift parameters

private:
  double alpha1, alpha2, alpha3, alpha4, alpha5;
};


}  // namespace amcl

#endif  // NAV2_UTIL__SENSORS__ODOM_H_
