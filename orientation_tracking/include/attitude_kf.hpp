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

class AttitudeKalmanFilterBase {
 public:
  AttitudeKalmanFilterBase(const Eigen::Vector4d &init_state)
      : q_(init_state),
        delta_q_(1, 0, 0, 0),
        bias_(0, 0, 0),
        delta_b_(0, 0, 0) {}

  virtual bool Propagate(const Eigen::Vector3d &omega, double delta_t) = 0;

  virtual bool GetCurrentState(Eigen::Vector4d &state) const = 0;

 protected:
  // orientation quaternion, [ x, y, z, w ]
  Eigen::Vector4d q_;
  // error state quaternion
  Eigen::Vector4d delta_q_;
  // bias of gyro
  Eigen::Vector3d bias_;
  // error state bias of gyro
  Eigen::Vector3d delta_b_;

  ImuIntegrator imu_integrator_;
};

class NominalStateFilter : AttitudeKalmanFilterBase {
 public:
  NominalStateFilter();

  bool Propagate(const Eigen::Vector3d &omega, double delta_t) override;

  bool GetCurrentState(Eigen::Vector4d &state) const override;

 private:
};

}  // vio

#endif  // ORIENTATION_TRACKING_ATTITUDE_KF_FILTER_HPP_
