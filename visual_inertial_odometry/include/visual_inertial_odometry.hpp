#ifndef VISUAL_INERTIAL_ODOMETRY_
#define VISUAL_INERTIAL_ODOMETRY_

#include <memory>
#include <mutex>
#include <thread>

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

  void ProcessNewImage(cv::Mat &img);

  void Start() {
    std::thread main_work(
        std::thread(&VisualInertialOdometry::ProcessDataInBuffer, this));
  }

 private:
  void InitializeFeatureTracker();

  void ProcessDataInBuffer();

  Status vio_status_;

  CameraModelPtr camera_;
  FeatureTrackerPtr feature_tracker_;

  // TODO: Use ROS for now.
  VIODataBuffer data_buffer_;
};

}  // vio

#endif  // VISUAL_ODOMETRY_
