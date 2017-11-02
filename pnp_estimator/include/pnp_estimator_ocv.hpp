#include "pnp_estimator.hpp"

namespace vio {

class PnPEstimatorOCV : public PnPEstimator {
 public:
  PnPEstimatorOCV() {}

  bool EstimatePose(const std::vector<cv::Point2f> &image_points,
                    const std::vector<cv::Point3f> &points3d, const cv::Mat &K,
                    std::vector<bool> &inliers, cv::Mat &R_est,
                    cv::Mat &t_est) override;
};

}  // vio
