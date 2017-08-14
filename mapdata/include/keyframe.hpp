#ifndef VIO_KEYFRAME_
#define VIO_KEYFRAME_

#include <memory>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "image_frame.hpp"

namespace vio {

class FramePose {
 public:
  FramePose() : timestamp(0) {
    position << 0, 0, 0;
    orientation << 0, 0, 0, 1;
  }

  // TODO: Will forget set timestamp.
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Vector4d orientation;
};

class Keyframe {
 public:
  // TODO: Frame no safe
  Keyframe(std::unique_ptr<ImageFrame> frame) : pose_inited_(false) {
    unique_frame_id_++;
    frame_id_ = unique_frame_id_;

    image_frame_ = std::move(frame);
  }

  Keyframe() = delete;

  int frame_id() const { return frame_id_; };

  const ImageFrame &image_frame() const { return *image_frame_; }

  void SetPose(const FramePose &pose) {
    pose_ = pose;
    pose_inited_ = true;
  }

/*
  void set_pose(const cv::Mat &R, const cv::Mat &t) {
    R.copyTo(pose_.R);
    t.copyTo(pose_.t);
  }
  const FramePose &pose() const { return pose_; }
  const cv::Mat &GetRot() const { return pose_.R; };
  const cv::Mat &GetT() const { return pose_.t; };
*/

  bool pose_inited() const { return pose_inited_; };
  void set_pose_inited(bool inited) { pose_inited_ = inited; }

 private:
  static int unique_frame_id_;
  int frame_id_;

  std::unique_ptr<ImageFrame> image_frame_;

  FramePose pose_;
  bool pose_inited_;
};

}  // namespace vio

#endif
