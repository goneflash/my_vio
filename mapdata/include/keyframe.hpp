#ifndef VIO_KEYFRAME_
#define VIO_KEYFRAME_

#include <memory>

#include <opencv2/opencv.hpp>

#include "image_frame.hpp"

namespace vio {

struct FramePose {
  cv::Mat R, t;
};

/*
struct FeatureMeasurement {
  Eigen::Vector2f keypoint;
  // TODO: Change to Eigen.
  cv::Mat descriptor;
};
*/

class Keyframe {
 public:
  Keyframe(std::unique_ptr<ImageFrame> frame) : pose_inited_(false) {
    unique_frame_id_++;
    frame_id_ = unique_frame_id_;

    image_frame_ = std::move(frame);
    // TODO: Not transfer ImageFrame, just copy the keypoints and descriptor.
  }

  Keyframe() = delete;

  size_t frame_id() const { return frame_id_; };

  const ImageFrame &image_frame() const { return *image_frame_; }

  void set_pose(const cv::Mat &R, const cv::Mat &t) {
    R.copyTo(pose_.R);
    t.copyTo(pose_.t);
  }
  const FramePose &pose() const { return pose_; }
  const cv::Mat &GetRot() const { return pose_.R; };
  const cv::Mat &GetT() const { return pose_.t; };

  bool pose_inited() const { return pose_inited_; };
  void set_pose_inited(bool inited) { pose_inited_ = inited; }

 private:
  size_t frame_id_;
  size_t pre_frame_id_;

  // Each frame will have a unique Id when initialized.
  static int unique_frame_id_;

  std::unique_ptr<ImageFrame> image_frame_;

  FramePose pose_;
  bool pose_inited_;
};

}  // namespace vio

#endif
