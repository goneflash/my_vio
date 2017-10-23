#ifndef VIO_KEYFRAME_
#define VIO_KEYFRAME_

#include <memory>

#include <opencv2/opencv.hpp>

#include "image_frame.hpp"
#include "mapdata_types.hpp"

namespace vio {

class Keyframe {
 public:
  Keyframe(std::unique_ptr<ImageFrame> frame)
      : frame_id_(CreateNewId<KeyframeId>()), pose_inited_(false) {
    image_frame_ = std::move(frame);
    // TODO: Not transfer ImageFrame, just copy the keypoints and descriptor.
  }

  Keyframe() = delete;

  KeyframeId frame_id() const { return frame_id_; };

  const ImageFrame &image_frame() const { return *image_frame_; }

 private:
  KeyframeId frame_id_;
  KeyframeId pre_frame_id_;

  // Each frame will have a unique Id when initialized.
  static int unique_frame_id_;

  std::unique_ptr<ImageFrame> image_frame_;

  // FramePose pose_;
  bool pose_inited_;
};

}  // namespace vio

#endif
