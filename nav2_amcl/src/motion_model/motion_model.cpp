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

#include "nav2_amcl/motion_model/motion_model.hpp"
#include <pluginlib/class_loader.hpp>
#include <string>

namespace nav2_amcl
{
std::shared_ptr<nav2_amcl::MotionModel>
MotionModel::createMotionModel(
  std::string & type, double alpha1, double alpha2,
  double alpha3, double alpha4, double alpha5)
{

   pluginlib::ClassLoader<nav2_amcl::MotionModel> poly_loader("nav2_amcl", "nav2_amcl::MotionModel");
   try
   {
   
   std::shared_ptr<nav2_amcl::MotionModel> motion_model_= poly_loader.createSharedInstance(type);
   motion_model_->initialize(alpha1, alpha2, alpha3, alpha4, alpha5);
   return motion_model_;
   }
   
   catch(pluginlib::PluginlibException& ex)
  {
    
    return nullptr;
  }

}

} // namespace nav2_amcl
