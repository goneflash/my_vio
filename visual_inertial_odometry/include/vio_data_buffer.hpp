#ifndef VIO_DATA_BUFFER_HPP_
#define VIO_DATA_BUFFER_HPP_

#include <iostream>

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "thread_safe_queue.hpp"

namespace vio {

/*
 * Used to hold the data (image & imu data) for processing.
 * Will drop data if the processing is slow.
 */
struct VIODataBufferStats {
 public:
  VIODataBufferStats()
      : received_count(0), dropped_count(0), left_in_buffer(0) {}
  int received_count;
  int dropped_count;
  int left_in_buffer;

  void Print() const {
    std::cout << "Received " << received_count << ", dropped " << dropped_count
              << ", still " << left_in_buffer << " in buffer " << std::endl;
  }
};

typedef std::chrono::high_resolution_clock::time_point TimeStamp;

struct TimeStampedImage {
  TimeStampedImage() {}
  TimeStampedImage(cv::Mat &img, TimeStamp timestamp)
      : image(img), time_stamp(timestamp) {}
  cv::Mat image;
  TimeStamp time_stamp;
};

class VIODataBuffer {
 public:
  // TODO: Move buffer size as parameters?
  VIODataBuffer()
      : buffer_closed_(false),
        skip_every_num_frames_(3),
        cur_skipped_count_(0),
        skip_time_interval_(50),
        image_buffer_size_(30),
        imu_buffer_size_(50) {
    // TODO: Not really correct.
    last_image_timestamp_ = std::chrono::high_resolution_clock::now();
  }

  void CloseBuffer() {
    buffer_closed_ = true;
    size_t size = 0;
    auto tmp_lock = image_buffer_.size(size);
    image_buffer_stats_.left_in_buffer = size;
  }

  void AddImageData(cv::Mat &img) {
    image_buffer_stats_.received_count++;

    if (SkipThisImage()) {
      cur_skipped_count_++;
      image_buffer_stats_.dropped_count++;
      std::cout << "Skipped an image.\n";
      return;
    }
    cur_skipped_count_ = 0;

    while (true) {
      size_t size = 0;
      auto tmp_lock = image_buffer_.size(size);
      if (size >= image_buffer_size_) {
        // TODO: Drop smartly.
        image_buffer_.Pop(std::move(tmp_lock));
        image_buffer_stats_.dropped_count++;
      } else {
        tmp_lock.unlock();
        last_image_timestamp_ = std::chrono::high_resolution_clock::now();
        TimeStampedImage new_image(img, last_image_timestamp_);
        image_buffer_.Push(new_image);
        break;
      }
    }
  }

  void AddImuData() {}

  // Return true if buffer has ended.
  bool GetImageDataOrEndOfBuffer(cv::Mat &image) {
    TimeStampedImage timed_image;
    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (buffer_closed_) return true;
    } while (!image_buffer_.TryPop(timed_image));
    image = timed_image.image;
    const auto interval = last_image_timestamp_ - timed_image.time_stamp;
    std::cout << "There's a lag in processing images: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(interval)
                     .count() << " ms.\n";
    return false;
  }

  // TODO: Interpolate imu data to get synchronous data.
  bool GetLatestDataComb() {}

  const VIODataBufferStats &image_buffer_stats() const {
    return image_buffer_stats_;
  }

 private:
  bool SkipThisImage() {
    if (cur_skipped_count_ >= skip_time_interval_) return false;
    const auto interval =
        std::chrono::high_resolution_clock::now() - last_image_timestamp_;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(interval)
            .count() > skip_time_interval_)
      return false;
    return true;
  }

  std::atomic<bool> buffer_closed_;

  // Image data buffer.
  int image_buffer_size_;
  ThreadSafeQueue<TimeStampedImage> image_buffer_;
  // TODO: Do not need mutex until it is used in multithread.
  VIODataBufferStats image_buffer_stats_;
  // Will drop new images if it met both of the criterions.
  // 1. skipped less than 3 frames.
  // 2. skipped in last 30 ms.
  int skip_every_num_frames_;
  int cur_skipped_count_;
  float skip_time_interval_;
  TimeStamp last_image_timestamp_;

  // IMU data buffer.
  int imu_buffer_size_;
  // ThreadSafeQueue<cv::Mat> imu_buffer_;
  VIODataBufferStats imu_buffer_stats_;
};

}  // vio

#endif  // VIO_DATA_BUFFER_HPP_
