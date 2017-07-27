#include "camera_model.hpp"

namespace vio {

template <typename ParamsType>
bool PinholeCameraModel<ParamsType>::ProjectPointToImagePlane(
    const Eigen::Vector3d &point, Eigen::Vector2d &pixel) const {
  // TODO: Eigen could use both () and [] to access elements?
  if (point(2) <= 0.0) {
    return false;
  }

  const ParamsType fx = params_[0];
  const ParamsType fy = params_[1];
  const ParamsType cx = params_[2];
  const ParamsType cy = params_[3];

  pixel(0) = fx * (point(0) / point(2)) + cx;
  pixel(1) = fy * (point(1) / point(2)) + cy;

  return true;
}

} // namespace vio
