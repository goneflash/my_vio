#ifndef CAMERA_MODEL_HPP_
#define CAMERA_MODEL_HPP_

#include <Eigen/Dense>

namespace vio {
/*
 * Use CRTP pattern.
 */
template<class DerivedCameraModel, typename ParamsType,
         std::size_t NumParams>
class CameraModel {
 public:
  typedef Eigen::Array<ParamsType, 4, 1> ParamsArray;

  CameraModel(
      int image_height, int image_width, const ParamsArray &params)
      : image_height_(image_height),
        image_width_(image_width) {
    // TODO: Add CHECK_EQ params.rows() NumParams
    params_ = params;
  }

  // Return false is the point is out of camera's view, e.g. behind the camera.
  // TODO: Use Eigen::Ref<> for reference type of Eigen.
  bool ProjectPoint(const Eigen::Vector3d &point,
                    Eigen::Vector2d &pixel) const {
    return static_cast<const DerivedCameraModel *>(this)->ProjectPointToPixel(
        point, pixel);
  }

  bool ProjectPointToPixel(const Eigen::Vector3d &point,
                           Eigen::Vector2d &pixel) const {
    return false;
  }

  int image_height() const { return image_height_; }
  int image_width() const { return image_width_; }

 protected:
  int image_height_;
  int image_width_;

  // Parameters of camera model.
  Eigen::Array<ParamsType, NumParams, 1> params_;
};

/*
 * Pinhole Model.
 * K = [ fx  0 cx
 *        0 fy cy
 *        0  0  1 ]
 * Number of parameters is 4: [fx, fy, cx, cy].
 *
 * Here in the template parameter list, must use PinholeCameraModel<ParamsType>.
 */
template <typename ParamsType>
class PinholeCameraModel :
  public CameraModel<PinholeCameraModel<ParamsType>, ParamsType, 4> {
 public:
  // using typename CameraModel<PinholeCameraModel, ParamsType, 4>::ParamsArray;
  typedef CameraModel<PinholeCameraModel, ParamsType, 4> CameraModelType;
  using CameraModelType::params_;
  using typename CameraModelType::ParamsArray;

  PinholeCameraModel(
      int image_height, int image_width, const ParamsArray &params)
      : CameraModelType(image_height, image_width, params) {}

  bool ProjectPointToPixel(const Eigen::Vector3d &point,
                           Eigen::Vector2d &pixel) const;

 private:
};

template <typename ParamsType>
bool PinholeCameraModel<ParamsType>::ProjectPointToPixel(
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

#endif
