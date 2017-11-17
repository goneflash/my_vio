#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace vio {

struct BasicReprojectionError {
  BasicReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T *const camera_intrinsics, const T *const camera_R_t,
                  const T *const point, T *residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera_R_t, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera_R_t[3];
    p[1] += camera_R_t[4];
    p[2] += camera_R_t[5];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // TODO: Apply distortion

    // Focal length
    const T focal = camera_intrinsics[0];
    const T principal_x = camera_intrinsics[1];
    const T principal_y = camera_intrinsics[2];

    // Compute final projected point position.
    T predicted_x = focal * xp + principal_x;
    T predicted_y = focal * yp + principal_y;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<BasicReprojectionError, 2, 3, 6, 3>(
        new BasicReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};

}  // vio
