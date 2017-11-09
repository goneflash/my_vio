#ifndef CAMERA_MODEL_HPP_
#define CAMERA_MODEL_HPP_

#include <memory>

#include <Eigen/Core>
// Only used for loading a configuration file.
#include <opencv2/opencv.hpp>

namespace vio {

enum CameraModelTypeName { UNKNOWN = 0, PINHOLE, FOV, FISHEYE };

#define PINHOLE_NUM_PARAMETERS 4
#define FOV_NUM_PARAMETERS 8
#define FISHEYE_NUM_PARAMETERS 8


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

typedef std::unique_ptr<CameraModel> CameraModelPtr;

CameraModelPtr CreateCameraModelFromConfig(const cv::FileNode &node);
// TODO: Maybe create from parameters.

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
    : public CameraModelBase<PinholeCameraModel<ParamsType>, ParamsType,
                             PINHOLE_NUM_PARAMETERS> {
 public:
  typedef CameraModelBase<PinholeCameraModel, ParamsType, 4> CameraModelType;
  using CameraModelType::params_;
  using typename CameraModelType::ParamsArray;

  PinholeCameraModel(int image_height, int image_width,
                     const ParamsArray &params)
      : CameraModelType(image_height, image_width, params) {
    CameraModelType::camera_type_ = PINHOLE;
  }
  PinholeCameraModel() = delete;

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

template <typename ParamsType>
class FisheyeCameraModel
    : public CameraModelBase<FisheyeCameraModel<ParamsType>, ParamsType,
                             FISHEYE_NUM_PARAMETERS> {
 public:
  typedef CameraModelBase<FisheyeCameraModel, ParamsType,
                          FISHEYE_NUM_PARAMETERS> CameraModelType;
  using CameraModelType::params_;
  using typename CameraModelType::ParamsArray;

  FisheyeCameraModel(int image_height, int image_width,
                     const ParamsArray &params)
      : CameraModelType(image_height, image_width, params) {
    CameraModelType::camera_type_ = FISHEYE;
  }

  bool ProjectPointToPixel(const Eigen::Vector3d &point,
                           Eigen::Vector2d &pixel) const override;

  const ParamsArray params() const { return params_; }

 private:
};

template <typename ParamsType>
bool FisheyeCameraModel<ParamsType>::ProjectPointToPixel(
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

  const ParamsType k1 = params_[4];
  const ParamsType k2 = params_[5];
  const ParamsType p1 = params_[6];
  const ParamsType p2 = params_[7];

  ParamsType undistorted_x = fx * (point(0) / point(2)) + cx;
  ParamsType undistorted_y = fy * (point(1) / point(2)) + cy;
  ParamsType r_square =
      undistorted_x * undistorted_x + undistorted_y * undistorted_y;

  pixel(0) = undistorted_x * (1 + k1 * r_square + k2 * r_square * r_square) +
             2 * p1 * undistorted_x * undistorted_y +
             p2 * (r_square + 2 * undistorted_x * undistorted_x);
  pixel(1) = undistorted_y * (1 + k1 * r_square + k2 * r_square * r_square) +
             2 * p2 * undistorted_x * undistorted_y +
             p1 * (r_square + 2 * undistorted_y * undistorted_y);

  if (pixel(0) < 0 || pixel(0) >= CameraModelType::image_width() ||
      pixel(1) < 0 || pixel(1) >= CameraModelType::image_height())
    return false;

  return true;
}

}  // namespace vio

#endif
