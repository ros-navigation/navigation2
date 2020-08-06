#ifndef NAV2_LOCALIZATION__MATCHER_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER_BASE_HPP_

#include <map.hpp>
#include <measurement.hpp>
#include <particle.hpp>
#include <vector>

namespace nav2_localization_base
{
    class Matcher2d
    {
        public:
            virtual float get_overlap(const nav2_localization::measurement_t& measurement, const pose_2d_t& pos, const nav2_localization::ndt_grid_2d_t& map); // Computes the overlap between a measurement and what should be measured given the current position and a map

        protected:
            Matcher2d(){}
    };  
};

#endif // NAV2_LOCALIZATION__MATCHER_BASE_HPP_
