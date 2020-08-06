#ifndef NAV2_LOCALIZATION__MEASUREMENT_HPP_
#define NAV2_LOCALIZATION__MEASUREMENT_HPP_

#include <vector>

namespace nav2_localization
{
	typedef struct
	{
		// TODO - How to represent keypoints?
		std::vector<std::vector<float>> keypoints; // Vector of 3D points
		// TODO - How to represent descriptors?? Plugin??
		std::vector<std::vector<float>> kps_descriptors // Vector of descriptors
	} measurement_t;
};

#endif // NAV2_LOCALIZATION__MEASUREMENT_HPP_
