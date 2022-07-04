#ifndef NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_
#define NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_

#include <vector>

namespace nav2_amcl
{

typedef struct {
    double x;
    double y;
    double yaw;

    double cov_matrix[9];
    double eigen_matrix[9];

    double time_ns; // time the measurement was made
} external_pose_t;

class ExternalPoseBuffer {

public:

// ExternalPoseBuffer():buffer_({}){

// }

// TODO: allocate memory for max_storage_size_ elements

/* 
* @brief Add new measurement to the storage (queue internally). If queue size grown larger than max_storage_size_, pop oldest measurement
* @param measurement external pose measurement
* @return
*/
void addMeasurement(const external_pose_t measurement);

/* 
* @brief Find closest external pose measurement to the query time, but not further than threshold
* @param query_time_ns query time, nanoseconds
* @param out_measurement found closest measurement
* @return True if closest measurement found
*/
bool findClosestMeasurement(double query_time_ns, external_pose_t& out_measurement) const;

private:

// how large can be the time difference between corresponding external pose measurement and laser scan
const uint32_t search_tolerance_ns_ = 0.5 * 1e9;

const double max_buff_size_ = 30;
std::vector<external_pose_t> buffer_ = {};

}; 



}


#endif // NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_