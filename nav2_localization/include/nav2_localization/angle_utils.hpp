#ifndef NAV2_LOCALIZATION__ANGLE_UTILS_HPP_
#define NAV2_LOCALIZATION__ANGLE_UTILS_HPP_

namespace nav2_localization
{
/**
 * @class AngleUtils
 * @brief Encapsulates useful methods to manipulate angles
 */
class AngleUtils
{
public:
    /**
     * @brief Calculates the difference between two angles and bounds the output to [-pi, pi]
     * @param a First angle
     * @param b Second angle
     * @return the difference between angle "a" and "b", bounded to [-pi, pi]
     */
    static double angleDiff(const double &a, const double &b);
};
}

#endif // NAV2_LOCALIZATION__ANGLE_UTILS_HPP_
