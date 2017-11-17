#include "graph_optimizer.hpp"

#include <iostream>
#include <vector>

#include "ceres/ceres.h"

#include "reprojection_error.hpp"

namespace vio {

class GraphOptimizerCeres : public GraphOptimizer {
 public:
  GraphOptimizerCeres() {}

  bool Optimize(const cv::Mat &K, std::vector<cv::Mat> &Rs,
                std::vector<cv::Mat> &ts, std::vector<cv::Point3f> &points,
                const std::vector<int> &obs_camera_idx,
                const std::vector<int> &obs_point_idx,
                const std::vector<cv::Vec2d> &obs_feature) override;

 private:
  bool ConstructProblem(const cv::Mat &K, const std::vector<cv::Mat> &Rs,
                        const std::vector<cv::Mat> &ts,
                        const std::vector<cv::Point3f> &points);

  bool AssignOptimizedResult(std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &ts,
                             std::vector<cv::Point3f> &points);
  void CheckPointsValid();

  void MatRotToAngleAxis(const cv::Mat &R, double *axis_angle);
  void AngleAxisToMatRot(const double *axis_angle, cv::Mat &R);

  std::vector<double> camera_intrinsics_;
  std::vector<std::vector<double> > camera_R_t_;
  std::vector<double> points_;
};

}  // vio
