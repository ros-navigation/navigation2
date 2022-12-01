#ifndef NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_
#define NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_

#include <vector>

namespace nav2_amcl
{

struct ExternalPoseMeasument {
    double x;
    double y;
    double yaw;

    double cov_matrix[9];
    double eigen_matrix[9];

    double time_sec; // time the measurement was made
};

class ExternalPoseBuffer {

public:

// TODO: allocate memory for max_storage_size_ elements
ExternalPoseBuffer(double search_tolerance_sec): search_tolerance_sec_(search_tolerance_sec) {}

/** 
* @brief Add new measurement to the buffer. If buffer size grown larger than max_storage_size_, pop oldest measurement
* @param measurement external pose measurement
* @return
*/
void addMeasurement(const ExternalPoseMeasument measurement);

/** 
* @brief Find closest external pose measurement to the query time, but not further than threshold
* @param query_time_sec time to look closest external pose for i.e. latest LaserScan reading time
* @param out_measurement found closest measurement
* @return True, if closest measurement found, False othervise
*/
bool findClosestMeasurement(double query_time_sec, ExternalPoseMeasument& out_measurement) const;

private:

// how large can be the time difference between corresponding external pose measurement and laser scan
const double search_tolerance_sec_;

const size_t max_buff_size_ = 10;
std::vector<ExternalPoseMeasument> buffer_ = {};

}; 



}


#endif // NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_