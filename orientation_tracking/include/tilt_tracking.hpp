#ifndef TILT_TRACKING_
#define TILT_TRACKING_

#include <math.h>

#include <Eigen/Dense>

#include "imu_integrator.hpp"

namespace vio {

// TODO: Don't need a class for that.

/*
 * Coordinate is Right-handed:
 *   X --> Thumb --> Right
 *   Y --> Up
 *   Z --> Towards me
 *
 *       Y / Yaw
 *       ^
 *       |
 *       |
 *       |______> X / Pitch
 *     /
 *    /
 *   /
 *  Z / Roll
 *
 *  Followed this tutorial:
 *  EE 267 Virtual Reality
 *  Course Notes: 3-DOF Orientation Tracking with IMUs
 *
 *
 *  Quaternion is in the order: x, y, z, w
 */
template <typename TYPE>
class TiltTracker {
 public:
  TiltTracker() : g_(9.81), alpha_(0.5) {}

  // Acceleration is represented:
  //     m/s^2
  // Roll and pitch is represented:
  //     radian
  bool GetRollPitch(TYPE acc_x, TYPE acc_y, TYPE acc_z, TYPE &roll,
                    TYPE &pitch);

  // From paragraph 5.2.
  // Equation (23)
  // Format is : [w x y z]
  bool GetCorrectionQuaternion(TYPE acc_x, TYPE acc_y, TYPE acc_z,
                               Eigen::Vector4d &q_correct);

  // From paragraph 5.3.
  bool CorrectOrientationInTilt(TYPE acc_x, TYPE acc_y, TYPE acc_z,
                                const Eigen::Vector4d &q_cur,
                                Eigen::Vector4d &q_correct);

  void set_tilt_correction_alpha(double alpha) { alpha_ = alpha; }

 private:
  TYPE g_;

  // Parameter for complimentary filter.
  double alpha_;

  ImuIntegrator imu_integrator_;
};

template <typename TYPE>
bool TiltTracker<TYPE>::GetRollPitch(TYPE acc_x, TYPE acc_y, TYPE acc_z,
                                     TYPE &roll, TYPE &pitch) {
  // Convert to use G as unit.
  const TYPE acc_x_g = acc_x / g_;
  const TYPE acc_y_g = acc_y / g_;
  const TYPE acc_z_g = acc_z / g_;

  roll = -atan2(-acc_x_g, acc_y_g);

  if (acc_y_g < 0.0)
    pitch = -atan2(-acc_z_g,
                   -acc_y_g * sqrt(acc_x_g * acc_x_g + acc_y_g * acc_y_g));
  else
    pitch =
        -atan2(-acc_z_g, acc_y_g * sqrt(acc_x_g * acc_x_g + acc_y_g * acc_y_g));

  return true;
}

template <typename TYPE>
bool TiltTracker<TYPE>::GetCorrectionQuaternion(TYPE acc_x, TYPE acc_y,
                                                TYPE acc_z,
                                                const Eigen::Vector4d &q_cur
                                                    Eigen::Vector4d &q_tilt) {
  // See chapter 5.3
  // Get acceleration quaternion in body frame.
  // TODO: The order of w, x, y, z
  Eigen::Vector4d q_acc_body(acc_x, acc_y, acc_z, 0);
  Eigen::Vector4d q_acc_body = q_acc_body.normalized();

  // Get acceleration quaternion in world frame.

  Eigen::Vector3d acc(acc_x, acc_y, acc_z);
  Eigen::Vector3d normalized_acc = acc.normalized();
  Eigen::Vector3d rotation_vector(-normalized_acc(2), 0, normalized_acc(0));

  q_tilt =
      vector_angle_to_quaternion(arccos(normalized_acc(1)), rotation_vector);
  return true;
}

template <typename TYPE>
bool TiltTracker<TYPE>::CorrectOrientationInTilt(TYPE acc_x, TYPE acc_y,
                                                 TYPE acc_z,
                                                 const Eigen::Vector3d &omega,
                                                 double delta_t,
                                                 const Eigen::Vector4d &q_cur,
                                                 Eigen::Vector4d &q_corrected) {
  // Propagate gyro data.
  // In this course, they use delta_q * p * delta_q_inverse to apply the
  // rotation which is not correct based on IMU model.
  Eigen::Vector4d updated_q;
  imu_integrator_.ZerothOrderIntegration(q_cur, omega, delta_t, updated_q);

  Eigen::Vector4d q_tilt_correction;
  if (!GetCorrectionQuaternion(acc_x, acc_y, acc_z, q_tilt_correction))
    return false;

  return true;
}

}  // vio

#endif  // TILT_TRACKING_
