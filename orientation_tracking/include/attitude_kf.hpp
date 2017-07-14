#ifndef ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_
#define ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace vio {
/*
 * Follows paper: "Indirect Kalman filter for 3D Attitude Estimation".
 * State has 7 elements, 4 for quaternion, 3 for gyro bias
 * x(t) = [ q(t)
 *          b(t) ]
 * Note that :
 * Quarternion in the has the format   : [ x y z w ]
 * Quarternion in Eighe has the format : [ w x y z ]
 *
 *
 */
class AttitudeKalmanFilter {
 public:
  AttitudeKalmanFilter(const Eigen::Quaterniond &init_state)
      : q_(init_state) {}
 private:
  // orientation quaternion
  Eigen::Quaterniond q_;
  // bias of gyro
  Eigen::Vector3d b_;
};

} // vio

#endif // ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_
