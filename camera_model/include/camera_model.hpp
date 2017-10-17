#ifndef CAMERA_MODEL_HPP_
#define CAMERA_MODEL_HPP_

#include <Eigen/Dense>

namespace vio {

enum CameraModelTypeName { UNKNOWN = 0, PINHOLE, FISHEYE };
/*
 * Abstract class for camera.
 */
class CameraModel {
 public:
  virtual bool ProjectPointToPixel(const Eigen::Vector3d &point,
                                   Eigen::Vector2d &pixel) const = 0;

  virtual int image_height() const = 0;
  virtual int image_width() const = 0;
  virtual CameraModelTypeName camera_model_type() const = 0;
};

/*
 * Use CRTP pattern.
 */
template <class DerivedCameraModel, typename ParamsType, std::size_t NumParams>
class CameraModelBase : public CameraModel {
 public:
  typedef Eigen::Array<ParamsType, NumParams, 1> ParamsArray;

  CameraModelBase(int image_height, int image_width, const ParamsArray &params)
      : image_height_(image_height),
        image_width_(image_width),
        camera_type_(UNKNOWN) {
    // TODO: Add CHECK_EQ params.rows() NumParams
    params_ = params;
  }

  // Return false is the point is out of camera's view, e.g. behind the camera.
  // TODO: Use Eigen::Ref<> for reference type of Eigen.
  bool ProjectPoint(const Eigen::Vector3d &point,
                    Eigen::Vector2d &pixel) const {
    return static_cast<const DerivedCameraModel *>(this)
        ->ProjectPointToPixel(point, pixel);
  }

  bool SetParams(const ParamsArray &params) { params_ = params; }

  int image_height() const { return image_height_; }
  int image_width() const { return image_width_; }
  CameraModelTypeName camera_model_type() const { return camera_type_; }

 protected:
  int image_height_;
  int image_width_;

  CameraModelTypeName camera_type_;
  // Parameters of camera model.
  ParamsArray params_;
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
class PinholeCameraModel
    : public CameraModelBase<PinholeCameraModel<ParamsType>, ParamsType, 4> {
 public:
  typedef CameraModelBase<PinholeCameraModel, ParamsType, 4> CameraModelType;
  using CameraModelType::params_;
  using typename CameraModelType::ParamsArray;

  PinholeCameraModel(int image_height, int image_width,
                     const ParamsArray &params)
      : CameraModelType(image_height, image_width, params) {
    CameraModelType::camera_type_ = PINHOLE;
  }

  bool ProjectPointToPixel(const Eigen::Vector3d &point,
                           Eigen::Vector2d &pixel) const override;

  const ParamsArray params() const { return params_; }

 private:
};

template <typename ParamsType>
bool PinholeCameraModel<ParamsType>::ProjectPointToPixel(
    const Eigen::Vector3d &point, Eigen::Vector2d &pixel) const {
  // TODO: Eigen could use both () and [] to access elements?
  // Point should not behind the camera center.
  if (point(2) <= 0.0) {
    return false;
  }

  const ParamsType fx = params_[0];
  const ParamsType fy = params_[1];
  const ParamsType cx = params_[2];
  const ParamsType cy = params_[3];

  pixel(0) = fx * (point(0) / point(2)) + cx;
  pixel(1) = fy * (point(1) / point(2)) + cy;

  if (pixel(0) < 0 || pixel(0) >= CameraModelType::image_width() ||
      pixel(1) < 0 || pixel(1) >= CameraModelType::image_height())
    return false;

  return true;
}

}  // namespace vio

#endif
