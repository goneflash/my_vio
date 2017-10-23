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
  enum VIOStatus {
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
  }

 private:
  void InitializeFeatureTracker();
  void ProcessDataInBuffer();
  void RunInitializer();

  bool KeepRunningMainWork() {
    // TODO: Should be atomic
    std::unique_lock<std::mutex> tmp_lock(running_main_work_mutex_);
    return running_main_work_;
  }

  std::mutex vio_status_mutex_;
  VIOStatus vio_status_;

  /*
   *  Functional objects.
   */
  CameraModelPtr camera_;
  FeatureTrackerPtr feature_tracker_;
  Keyframe *last_keyframe_;
  /*
   * Data structures.
   */
  VIODataBuffer data_buffer_;

  std::mutex keyframes_mutex_;
  Keyframes keyframes_;

  std::mutex landmarks_mutex_;
  Landmarks landmarks_;
  LandmarkStats landmark_stats;

  std::unique_ptr<std::thread> main_work_;
  std::mutex running_main_work_mutex_;
  bool running_main_work_;
};

void RemoveUnmatchedFeatures(Keyframe *frame);

bool ProcessMatchesToLandmarks(Keyframe *frame0, Keyframe *frame1,
                               const std::vector<cv::DMatch> &matches,
                               Landmarks &landmarks);

void RemoveShortTracks(Landmarks &landmarks, KeyframeId &cur_keyframe_id);

}  // vio

#endif  // VISUAL_ODOMETRY_
