#ifndef VIO_FEATURE_SET_
#define VIO_FEATURE_SET_

#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace vio {

class FeatureSet {
 public:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

}  // vio

#endif
