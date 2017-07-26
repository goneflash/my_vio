#include "attitude_kf.hpp"

namespace vio {

bool NominalStateFilter::Propagate(const Eigen::Vector3d &omega,
    double delta_t) {
  
  return true;
}

bool NominalStateFilter::GetCurrentState(Eigen::Vector4d &state) const {
  return true;
}

} // vio
