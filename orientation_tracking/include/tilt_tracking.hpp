#ifndef TILT_TRACKING_
#define TILT_TRACKING_

#include <math.h>

#include <Eigen/Dense>

namespace vio {

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
 */
template <typename TYPE>
class TiltTracker {
 public:
  TiltTracker() : g_(9.81) {}

  // Acceleration is represented:
  //     m/s^2
  // Roll and pitch is represented:
  //     radian
  bool GetRollPitch(TYPE acc_x, TYPE acc_y, TYPE acc_z, TYPE &roll,
                    TYPE &pitch);

 private:
  TYPE g_;
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

}  // vio

#endif  // TILT_TRACKING_
