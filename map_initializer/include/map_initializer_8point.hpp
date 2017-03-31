#include "map_initializer.hpp"

#include <opencv2/sfm.hpp>

namespace vio {

class MapInitializer8Point : public MapInitializer {
 public:
  explicit MapInitializer8Point(MapInitializerOptions option);
  ~MapInitializer8Point() {}

  virtual bool Initialize(
      const std::vector<std::vector<cv::Vec2d> > &feature_vectors,
      const cv::Mat &K, std::vector<cv::Point3f> &points3d,
      std::vector<bool> &points3d_mask, std::vector<cv::Mat> &Rs,
      std::vector<cv::Mat> &ts) override;

 protected:
  bool InitializeTwoFrames(const std::vector<cv::Vec2d> &kp0,
                           const std::vector<cv::Vec2d> &kp1, const cv::Mat &K,
                           std::vector<cv::Point3f> &points3d,
                           std::vector<bool> &points3d_mask,
                           std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &ts);

  // Find correct R, t combination.
  template <typename Point3Type>
  bool SelectSolutionRT(const std::vector<cv::Mat> &R,
                        const std::vector<cv::Mat> &t, const cv::Mat &K,
                        const std::vector<cv::Vec2d> &kp0,
                        const std::vector<cv::Vec2d> &kp1,
                        const std::vector<bool> &match_inliers, cv::Mat &R_best,
                        cv::Mat &t_best, std::vector<Point3Type> &points_3d,
                        std::vector<bool> &point3d_mask);
  int EvaluateSolutionRT(const cv::Mat &R, const cv::Mat &t, const cv::Mat &K,
                         const std::vector<cv::Vec2d> &kp0,
                         const std::vector<cv::Vec2d> &kp1,
                         const std::vector<bool> &match_inliers,
                         std::vector<cv::Point3f> &points_3d,
                         std::vector<bool> &points3d_mask);

  // Fundamental Matrix
  bool ComputeFundamental(const std::vector<cv::Vec2d> &kp0,
                          const std::vector<cv::Vec2d> &kp1, cv::Mat &F);
  // Handle n points, at least 8.
  bool ComputeFundamentalDLT(const std::vector<cv::Vec2d> &kp0,
                             const std::vector<cv::Vec2d> &kp1, cv::Mat &F);
  cv::Mat ComputeFundamentalOCV(const std::vector<cv::Vec2d> &kp0,
                                const std::vector<cv::Vec2d> &kp1);

  // Homograph Matrix

  template <typename Point3Type>
  void TriangulatePoints(const std::vector<cv::Vec2d> &kp0,
                         const std::vector<cv::Vec2d> &kp1, const cv::Mat &P1,
                         const cv::Mat &P2, std::vector<Point3Type> &point3d);

  // cv::Mat ComputeEfromF(const cv::Mat &F, const cv::Mat &K);
  void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

  bool use_f_ransac_;
  double f_ransac_confidence_;
  double f_ransac_max_dist_to_epipolar_;

  int min_triangluated_;
  double reprojection_error_thresh_;
  double parallex_thresh_;

  bool verbose_;
};

}  // vio
