#include "map_initializer.hpp"

#include <opencv2/sfm.hpp>

namespace vio {

class MapInitializerLIBMV : public MapInitializer {
 public:
  MapInitializerLIBMV() {}
  ~MapInitializerLIBMV() {}

  virtual bool Initialize(
      const std::vector<std::vector<cv::Vec2d> > &feature_vectors,
      const cv::Mat &K, std::vector<cv::Point3f> &points3d,
      std::vector<bool> &points3d_mask, std::vector<cv::Mat> &Rs,
      std::vector<cv::Mat> &ts) override;

 private:
  bool InitializeTwoFrames(const std::vector<cv::Vec2d> &kp0,
                           const std::vector<cv::Vec2d> &kp1, const cv::Mat &K,
                           std::vector<cv::Point3f> &points3d,
                           std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &ts);
};

}  // vio
