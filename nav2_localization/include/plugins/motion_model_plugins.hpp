#ifndef NAV2_LOCALIZATION__MOTION_MODEL_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MOTION_MODEL_PLUGINS_HPP_

#include <interfaces/motion_model_base.hpp>
#include <vector>

namespace nav2_localization_plugins
{
    // Implements a Bicycle motion model
    class Bicycle : public nav2_localization_base::MotionModel
    {   
        public:
            Bicycle(){}
			// TODO - How to send odometry?
    		std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
    };

    // Implements an Ackermann motion model
	//// TODO - Use this to simplify ackermann vehicle to bicycle model and then use the Bicycle model
    class Ackermann : public nav2_localization_base::MotionModel
    {   
        public:
            Ackermann(){}
			// TODO - How to send odometry?
    		std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
    };

    // Implements a Differential motion model
    class Differential : public nav2_localization_base::MotionModel
    {   
        public:
            Differential(){}
			// TODO - How to send odometry?
    		std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
    };  

    // Implements an Omnidirectional motion model
    class Omnidirectional : public nav2_localization_base::MotionModel
    {   
        public:
            Omnidirectional(){}
			// TODO - How to send odometry?
    		std::vector<float> get_transformation(const Odometry& odom); // Get the position and orientation transformation using a given odometry
    };  
};

#endif // NAV2_LOCALIZATION__MOTION_MODEL_PLUGINS_HPP_
