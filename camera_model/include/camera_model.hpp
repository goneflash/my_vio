#ifndef CAMERA_MODEL_HPP_
#define CAMERA_MODEL_HPP_

#include <memory>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vio {

enum CameraModelTypeName { UNKNOWN = 0, PINHOLE, FISHEYE };

#define FISHEYE_NUM_PARAMETERS 8

/*
 * Abstract class for camera.
 */
class CameraModel {
 public:
  virtual bool ProjectPointToPixel(const Eigen::Vector3d &point,
                                   Eigen::Vector2d &pixel) const = 0;
  virtual bool UndistortPixel(const Eigen::Vector2d &distorted,
                              Eigen::Vector2d &undistorted) const = 0;

  virtual int image_height() const = 0;
  virtual int image_width() const = 0;

  // TODO: Make sure should return the one after undistorted?
  /*
   * fx 0  cx
   * 0  fy cy
   * 0  0   1
   */
  virtual const cv::Mat &camera_matrix() const = 0;

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
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
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

  const ParamsArray params() const { return params_; }
  const cv::Mat &camera_matrix() const { return camera_matrix_; }
  CameraModelTypeName camera_model_type() const { return camera_type_; }

 protected:
  int image_height_;
  int image_width_;

  CameraModelTypeName camera_type_;
  // Parameters of camera model.
  ParamsArray params_;
  // Camera intrinsics matrix.
  cv::Mat camera_matrix_;
};

/*
 * Pinhole Model.
 * Camera Intrisics:
 * K = [ fx  0 cx
 *        0 fy cy
 *        0  0  1 ]
 * Distortion model parameters:
 *
 * Number of parameters is 8: [fx, fy, cx, cy, k1, k2, p1, p2].
 *
 * Here in the template parameter list, must use PinholeCameraModel<ParamsType>.
 */

#define PINHOLE_NUM_PARAMETERS 8

template <typename ParamsType>
class PinholeCameraModel
    : public CameraModelBase<PinholeCameraModel<ParamsType>, ParamsType,
                             PINHOLE_NUM_PARAMETERS> {
 public:
  typedef CameraModelBase<PinholeCameraModel, ParamsType,
                          PINHOLE_NUM_PARAMETERS> CameraModelType;

  using CameraModelType::params_;
  using typename CameraModelType::ParamsArray;

  PinholeCameraModel(int image_height, int image_width,
                     const ParamsArray &params)
      : CameraModelType(image_height, image_width, params) {
    CameraModelType::camera_type_ = PINHOLE;
    CameraModelType::camera_matrix_ = cv::Mat_<double>(3, 3) << params[0], 0.0,
    params[2], 0.0, params[1], params[3], 0.0, 0.0, 1.0;
  }

  PinholeCameraModel() = delete;

  bool ProjectPointToPixel(const Eigen::Vector3d &point,
                           Eigen::Vector2d &pixel) const override;
  bool UndistortPixel(const Eigen::Vector2d &distorted,
                      Eigen::Vector2d &undistorted) const override;

  // const ParamsArray params() const { return params_; }

 private:
  void initializeUndistortTable();
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

template <typename ParamsType>
bool PinholeCameraModel<ParamsType>::UndistortPixel(
    const Eigen::Vector2d &distorted, Eigen::Vector2d &undistorted) const {
  return true;
}

template <typename ParamsType>
void PinholeCameraModel<ParamsType>::initializeUndistortTable() {}

}  // namespace vio

#endif
