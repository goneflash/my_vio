#ifndef VISUAL_ODOMETRY_
#define VISUAL_ODOMETRY_

#include <memory>

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "feature_tracker.hpp"

#include "vio_data_buffer.hpp"

namespace vio {

class VisualInertialOdometry {
 public:
  enum Status {
    UNINITED = 0,
    INITED,
  };

  VisualInertialOdometry(CameraModelPtr camera);

  void ProcessNewImage(const cv::Mat &img);

  /*
  void AddImageData(cv::Mat &img) {
    data_buffer_.AddImageData(img);
  }
  */

  // TODO: When in real life cases. Create a buffer to hold images
  // and then process them.
  // TODO: Multi-threading.
  void ProcessDataInBuffer() {}

 private:
  void InitializeFeatureTracker();

  Status vio_status_;

  CameraModelPtr camera_;
  FeatureTrackerPtr feature_tracker_;

  // TODO: Use ROS for now.
  // VIODataBuffer data_buffer_;
};

}  // vio

#endif  // VISUAL_ODOMETRY_
