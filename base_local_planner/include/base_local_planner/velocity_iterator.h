/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#define DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#include <algorithm>
#include <cmath>
#include <vector>

namespace base_local_planner {

  /**
   * We use the class to get even sized samples between min and max, inluding zero if it is not included (and range goes from negative to positive
   */
  class VelocityIterator {
    public:
      VelocityIterator(double min, double max, int num_samples):
        current_index(0)
      {
        if (min == max) {
          samples_.push_back(min);
        } else {
          num_samples = std::max(2, num_samples);

          // e.g. for 4 samples, split distance in 3 even parts
          double step_size = (max - min) / double(std::max(1, (num_samples - 1)));

          // we make sure to avoid rounding errors around min and max.
          double current;
          double next = min;
          for (int j = 0; j < num_samples - 1; ++j) {
            current = next;
            next += step_size;
            samples_.push_back(current);
            // if 0 is among samples, this is never true. Else it inserts a 0 between the positive and negative samples
            if ((current < 0) && (next > 0)) {
              samples_.push_back(0.0);
            }
          }
          samples_.push_back(max);
        }
      }

      double getVelocity(){
        return samples_.at(current_index);
      }

      VelocityIterator& operator++(int){
        current_index++;
        return *this;
      }

      void reset(){
        current_index = 0;
      }

      bool isFinished(){
        return current_index >= samples_.size();
      }

    private:
      std::vector<double> samples_;
      unsigned int current_index;
  };
};
#endif
