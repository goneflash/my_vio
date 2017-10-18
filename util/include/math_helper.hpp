#ifndef UTIL_MATH_HELPER_HPP_
#define UTIL_MATH_HELPER_HPP_

#include <Eigen/Core>

// TODO: Need tests.

// Quaternion format : [x, y, z, w]
// q x p =
// [ q4p1 + q3p2 - q2p3 + q1p4
//  -q3p1 + q4p2 + q1p3 + q2p4
//   q2p1 - q1p2 + q4p3 + q3p4
//  -q1p1 - q2p2 - q3p3 + q4p4 ]
inline Eigen::Vector4d quaternion_multi(const Eigen::Vector4d &q,
                                        const Eigen::Vector4d &p) {
  Eigen::Vector4d result;
  result << q(3) * p(0) + q(2) * p(1) - q(1) * p(2) + q(0) * p(3),
      -q(2) * p(0) + q(3) * p(1) + q(0) * p(2) + q(1) * p(3),
      q(1) * p(0) - q(0) * p(1) + q(3) * p(2) + q(2) * p(3),
      -q(0) * p(0) - q(1) * p(1) + q(2) * p(2) + q(3) * p(3);
  return result;
}

/*
 * [q1, q2, q3]' -->
 * [  0 -q3  q2
 *   q3   0 -q1
 *  -q2  q1   0 ]
 */
inline Eigen::MatrixXd skew_symmetric(const Eigen::Vector3d &w) {
  Eigen::MatrixXd m(3, 3);
  m << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  return m;
}

/* w = [wx, wy, wz]
 * Omega(w) -->
 * [  0  wz -wy  wx
 *  -wz   0  wx  wy
 *   wy -wx   0  wz
 *  -wx -wy -wz   0 ]
 */
inline Eigen::MatrixXd omega_matrix(const Eigen::Vector3d &w) {
  Eigen::MatrixXd m(4, 4);
  m << 0, w(2), -w(1), w(0), -w(2), 0, w(0), w(1), w(1), -w(0), 0, w(2), -w(0),
      -w(1), -w(2), 0;
  return m;
}

#endif  // UTIL_MATH_HELPER_HPP_
