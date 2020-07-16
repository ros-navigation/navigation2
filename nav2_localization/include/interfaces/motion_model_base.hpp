#ifndef NAV2_LOCALIZATION__MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__MOTION_MODEL_BASE_HPP_

#include <vector>

namespace nav2_localization_base
{
    class MotionModel
    {   
        public:
			// TODO - How to send odometry?
            virtual std::vector<float> get_transformation(const Odometry& odom); // Applies a motion model plugin to a certain odometry

        protected:
            MotionModel(){}
    };  
};

#endif // NAV2_LOCALIZATION__MOTION_MODEL_BASE_HPP_
