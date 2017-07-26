#include "attitude_kf.hpp"

namespace vio {

 /* Follows paper: "Indirect Kalman filter for 3D Attitude Estimation".
  * Use equation 155.
  */
bool NominalStateFilter::Propagate(const Eigen::Vector3d &w,
    double delta_t) {
  const Eigen::Vector3d w_est = w - bias_;
  Eigen::Vector4d updated_q;
  imu_integrator_.zerothOrderIntegration(q_, w, delta_t, updated_q);

  q_ = updated_q;
  return true;
}

bool NominalStateFilter::GetCurrentState(Eigen::Vector4d &state) const {
  return true;
}

} // vio
