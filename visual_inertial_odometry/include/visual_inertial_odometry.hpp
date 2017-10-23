#ifndef VISUAL_INERTIAL_ODOMETRY_
#define VISUAL_INERTIAL_ODOMETRY_

#include <iostream>

#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "feature_tracker.hpp"
#include "feature_tracks.hpp"
#include "keyframe.hpp"
#include "mapdata_types.hpp"

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
    // TODO: Is this really necessary?
    std::unique_lock<std::mutex> tmp_lock(running_main_work_mutex_);
    running_main_work_ = true;
    tmp_lock.unlock();
    main_work_ = std::unique_ptr<std::thread>(
        new std::thread(&VisualInertialOdometry::ProcessDataInBuffer, this));
  }

  void Stop() {
    std::unique_lock<std::mutex> tmp_lock(running_main_work_mutex_);
    running_main_work_ = false;
    tmp_lock.unlock();

    main_work_->join();

    std::cout << "Total image: " << data_buffer_.image_total_num() << std::endl;
    std::cout << "Dropped image: " << data_buffer_.image_dropped_num()
              << std::endl;
  }

 private:
  void InitializeFeatureTracker();

  void ProcessDataInBuffer();

  bool KeepRunningMainWork() {
    // TODO: Should be atomic
    std::unique_lock<std::mutex> tmp_lock(running_main_work_mutex_);
    return running_main_work_;
  }

  Status vio_status_;

  /*
   *  Functional objects.
   */
  CameraModelPtr camera_;
  FeatureTrackerPtr feature_tracker_;

  /*
   * Data structures.
   */
  VIODataBuffer data_buffer_;
  Keyframes keyframes_;
  Landmarks landmarks_;

  std::unique_ptr<std::thread> main_work_;
  std::mutex running_main_work_mutex_;
  bool running_main_work_;
};

bool ConstructAndAddKeyframe(Keyframe &frame0, Keyframe &frame1,
                             const std::vector<cv::DMatch> &matches,
                             Landmarks &landmarks);

}  // vio

#endif  // VISUAL_ODOMETRY_
