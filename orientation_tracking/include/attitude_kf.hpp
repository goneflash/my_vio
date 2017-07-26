#ifndef ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_
#define ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_

#include <Eigen/Dense>
#include <iostream>

#include "imu_integrator.hpp"

namespace vio {
/*
 * Follows paper: "Indirect Kalman filter for 3D Attitude Estimation".
 * State has 7 elements, 4 for quaternion, 3 for gyro bias
 * x(t) = [ q(t)
 *          b(t) ]
 * Note that :
 * Quarternion in the paper has the format   : [ x y z w ]
 * Quarternion in Eigen has the format : [ w x y z ]
 *
 * Angular velocity (omega) format : [ wx wy wz ]
 *
 */
class AttitudeKalmanFilter {
 public:
  AttitudeKalmanFilter(const Eigen::Quaterniond &init_state)
      : q_(init_state) {}

  bool Propagate(const Eigen::Vector3d &omega, double delta_t);

  bool GetCurrentState(Eigen::Quaterniond &state) const;
 private:
  // orientation quaternion
  Eigen::Quaterniond q_;
  // error state quaternion
  Eigen::Quaterniond delta_q_;
  // bias of gyro
  Eigen::Vector3d b_;
  // error state bias of gyro
  Eigen::Vector3d delta_b_;
};

} // vio

#endif // ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_
