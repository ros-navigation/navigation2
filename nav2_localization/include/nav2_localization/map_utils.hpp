#ifndef NAV2_LOCALIZATION__MAP_UTILS_HPP_
#define NAV2_LOCALIZATION__MAP_UTILS_HPP_

#include <cstdint>
#include <utility>

// TODO: shoudl be moved to nav2_util?
namespace nav2_localization
{
class MapUtils
{
public:
    static int8_t coordinates_to_index(const uint32_t &x, const uint32_t &y, const uint32_t &map_width);
    static std::pair<uint32_t, uint32_t> index_to_coordinates(const int8_t &index, const uint32_t &map_width);
    static double distance_between_two_points(const uint32_t &x1, const uint32_t &y1,
                                              const uint32_t &x2, const uint32_t &y2);
};
}

#endif // NAV2_LOCALIZATION__MAP_UTILS_HPP_