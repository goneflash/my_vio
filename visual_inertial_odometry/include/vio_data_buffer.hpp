#ifndef VIO_DATA_BUFFER_HPP_
#define VIO_DATA_BUFFER_HPP_

#include <memory>
#include <mutex>
#include <queue>

namespace vio {

/*
 * Used to hold the data (image & imu data) for processing.
 * Will drop data if the processing is slow.
 */
class VIODataBuffer {
 public:
  VIODataBuffer()
      : image_buffer_size_(5),
        image_total_num_(0),
        image_dropped_num_(0),
        imu_buffer_size_(20),
        imu_total_num_(0),
        imu_dropped_num_(0) {}

  void AddImageData(cv::Mat &img) {
    image_total_num_++;
    std::lock_guard<std::mutex> buffer_guard(image_buffer_mutex_);
    while (image_buffer_.size() >= image_buffer_size_) {
      image_buffer_.pop();
      image_dropped_num_++;
    }
    // TODO: Use smart way to decide drop current image or the oldest image.
    // e.g. depend on the rotation and position difference.
    image_buffer_.push(img);
  }

  void AddImuData() {}

  /*
  cv::Mat GetImageData() {
    std::lock_guard<std::mutex> buffer_guard(image_buffer_mutex_);
    if (image_buffer_.empty())
      return nullptr;
    // TODO: What if no data is available?
    cv::Mat image = image_buffer_.front();
    image_buffer_.pop();
    return image;
  }
  */

  // TODO: Interpolate imu data to get synchronous data.
  bool GetLatestDataComb() {}

 private:
  // Image data buffer.
  int image_buffer_size_;
  int image_total_num_;
  int image_dropped_num_;

  std::mutex image_buffer_mutex_;
  std::queue<cv::Mat> image_buffer_;

  // IMU data buffer.
  int imu_buffer_size_;
  int imu_total_num_;
  int imu_dropped_num_;
};
}

#endif  // VIO_DATA_BUFFER_HPP_
