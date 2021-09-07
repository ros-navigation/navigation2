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

#ifndef NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include <string>
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"


namespace nav2_amcl
{

class MotionModel
{
public:
  
  void initialize(double alpha1, double alpha2,double alpha3, double alpha4, double alpha5,double seperation_between_the_wheels,std::string cmd_vel_topic)
    {

    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
    alpha5_ = alpha5;
    seperation_between_the_wheels_ = seperation_between_the_wheels;
    cmd_vel_topic_ = cmd_vel_topic;
    
}
  //virtual void initialize(double alpha1, double alpha2,double alpha3, double alpha4, double alpha5,double dimension,std::string cmd_vel_topic)=0;
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta)=0;
  virtual ~MotionModel(){}
  
  protected:
      double alpha1_;
      double alpha2_;
      double alpha3_;
      double alpha4_;
      double alpha5_;
      double seperation_between_the_wheels_;
      std:: string cmd_vel_topic_;
      MotionModel(){}

};
}
#endif  // NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
