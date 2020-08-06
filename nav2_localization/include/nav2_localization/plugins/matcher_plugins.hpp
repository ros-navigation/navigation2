#ifndef NAV2_LOCALIZATION__MATCHER_PLUGINS_HPP_
#define NAV2_LOCALIZATION__MATCHER_PLUGINS_HPP_

#include <interfaces/matcher_base.hpp>
#include <map.hpp>
#include <measurement.hpp>
#include <vector>

namespace nav2_localization_plugins
{
    // Implements a vanilla matcher for 2D NDT maps (for NDT-MCL localization)
    class VanillaNDT2d : public nav2_localization_base::Matcher
    {
		private:
            ndt_grid_2d_t map;

        public:
			VanillaNDT2d(){}

            // TODO - How to send position?
            float get_overlap(const nav2_localization::measurement_t& measurement, const vector<float>& pos, const nav2_localization::ndt_grid_2d_t& map); // Computes the overlap between a measurement and what should be measured given the current position and a map
    };  
};

#endif // NAV2_LOCALIZATION__MATCHER_PLUGINS_HPP_
