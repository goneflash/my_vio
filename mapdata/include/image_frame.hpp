#ifndef VIO_IMAGE_FRAME_
#define VIO_IMAGE_FRAME_

#include <opencv2/opencv.hpp>

namespace vio {

class ImageFrame {
 public:
  ImageFrame(){};
  ImageFrame(const cv::Mat &image);

  // Disabled for Uint test.
  // ImageFrame() = delete;

  const cv::Mat &GetImage() const { return image_; }

  void set_features(std::vector<cv::KeyPoint> &keypoints,
                    cv::Mat &descriptors) {
    set_keypoints(keypoints);
    set_descriptors(descriptors);
  }

  const std::vector<cv::KeyPoint> &keypoints() const { return keypoints_; }
  void set_keypoints(std::vector<cv::KeyPoint> &keypoints) {
    keypoints_ = std::move(keypoints);
    CreateGridKeypointIndex();
  }
  const cv::Mat &descriptors() const { return descriptors_; }
  void set_descriptors(cv::Mat &descriptors) {
    descriptors.copyTo(descriptors_);
  }

  void SetGridSize(int width, int height);
  bool GetNeighborKeypointsInRadius(const cv::KeyPoint &query,
                                    double dist_thresh,
                                    std::vector<int> &candidates_id) const;

 private:
  bool CreateGridKeypointIndex();

  cv::Mat image_;

  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

  // ------------ Put keypoints in bins ------------
  bool has_grid_keypoints_;
  // Number of pixel of the width of a grid
  int grid_width_size_;
  // Number of pixel of the height of a grid
  int grid_height_size_;
  // image_width / grid_width_size_
  int grid_width_index_range_;
  // image_height / grid_height_size_
  int grid_height_index_range_;
  // How many neighbor grids should be searched
  int grid_search_index_range_;

  typedef std::vector<int> KeypointIndexArry;
  std::vector<std::vector<KeypointIndexArry> > grid_keypoints_index_;

  struct BinCoord {
    int x, y;
  };
  std::vector<BinCoord> keypoint_bin_coords_;
};

}  // namespace vio

#endif
