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
    main_work_ = std::unique_ptr<std::thread>(
        new std::thread(&VisualInertialOdometry::ProcessDataInBuffer, this));
  }

  void Stop() {
    main_work_->join();
  }

 private:
  void InitializeFeatureTracker();

  void ProcessDataInBuffer();

  Status vio_status_;

  CameraModelPtr camera_;
  FeatureTrackerPtr feature_tracker_;

  VIODataBuffer data_buffer_;
  std::unique_ptr<std::thread> main_work_;
};

}  // vio

#endif  // VISUAL_ODOMETRY_
