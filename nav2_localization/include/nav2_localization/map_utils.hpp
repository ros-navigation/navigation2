#ifndef NAV2_LOCALIZATION__MAP_UTILS_HPP_
#define NAV2_LOCALIZATION__MAP_UTILS_HPP_

#include <cstdint>
#include <utility>

// TODO: should this be moved to nav2_util?
namespace nav2_localization
{
/**
 * @class MapUtils
 * @brief useful methods for occupancy grid maps
 */
class MapUtils
{
public:
    /**
     * @brief Returns the index of a point in the map
     * @param x
     * @param y
     * @param map_width
     * @return Index of (x, y) in the map
     */
    static int coordinatesToIndex(const uint32_t &x, const uint32_t &y, const uint32_t &map_width);

    /**
     * @brief Returns the coordinates of an index in the map
     * @param index 
     * @param map_width
     * @return The coordinates corresponding to the given index
     */
    static std::pair<uint32_t, uint32_t> indexToCoordinates(const int &index, const uint32_t &map_width);

    /**
     * @brief Returns the euclidean distance between two points
     * @param x1 x corrdinate of point 1
     * @param y1 y corrdinate of point 1
     * @param x2 x corrdinate of point 2
     * @param y2 y corrdinate of point 2
     * @return The euclidean distance between (x1, y1) and (x2, y2)
     */
    static double distanceBetweenTwoPoints(const uint32_t &x1, const uint32_t &y1,
                                           const uint32_t &x2, const uint32_t &y2);
};
}

#endif // NAV2_LOCALIZATION__MAP_UTILS_HPP_