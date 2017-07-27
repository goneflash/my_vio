#include "camera_model.hpp"

namespace vio {

template <typename ParamsType>
bool PinholeCameraModel<ParamsType>::ProjectPointToImagePlane(
    const Eigen::Vector3d &point, Eigen::Vector2d &pixel) const {
  return true;
}

} // namespace vio
