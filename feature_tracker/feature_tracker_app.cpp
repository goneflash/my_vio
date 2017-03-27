#include <iostream>
#include <string>

#include "gtest/gtest.h"

#include "feature_tracker.hpp"
#include "util.hpp"

using namespace std;
using namespace cv;
using namespace vio;

class Options {
 public:
  string path;
  string format;
  string detector;
  string descriptor;
};

int TestFramesInFolder(Options option) {
#ifndef __linux__
  cout << "Error: Test folder Not supported. Currently only support "
          "Linux.\n" return -1;
#endif
  vector<string> images;
  if (!GetImageNamesInFolder(option.path, option.format, images)) return -1;
  if (images.size() < 2) {
    cout << "Error: Find only " << images.size() << " images.\n";
    return -1;
  }
  cout << "Testing with " << images.size() << " images.\n";

  // Create tracker
  FeatureMatcherOptions matcher_options;
  std::unique_ptr<FeatureMatcher> matcher =
      FeatureMatcher::CreateFeatureMatcher(matcher_options);

  FeatureTrackerOptions tracker_options;
  if (option.detector.size())
    tracker_options.detector_type = option.detector;
  if (option.descriptor.size())
    tracker_options.descriptor_type = option.descriptor;
  FeatureTracker *tracker = FeatureTracker::CreateFeatureTracker(
      tracker_options, std::move(matcher));

  if (tracker == NULL)
    return -1;

  // Visualize tracking
  cv::namedWindow("result", cv::WINDOW_AUTOSIZE);

  // Track first frame.
  cv::Mat image0 = cv::imread(images[0]);
  std::unique_ptr<vio::ImageFrame> frame_pre(new vio::ImageFrame(image0));
  tracker->TrackFirstFrame(*frame_pre);

  for (int i = 1; i < images.size(); ++i) {
    cv::Mat image = cv::imread(images[i]);
    std::unique_ptr<vio::ImageFrame> frame_cur(new vio::ImageFrame(image));
    std::vector<cv::DMatch> matches;
    tracker->TrackFrame(*frame_pre, *frame_cur, matches);

    std::cout << "Found match " << matches.size() << std::endl;

    cv::Mat output_img = frame_cur->GetImage().clone();

    int thickness = 2;
    for (int i = 0; i < matches.size(); ++i) {
      line(output_img,
           frame_cur->keypoints()[matches[i].trainIdx].pt,
           frame_pre->keypoints()[matches[i].queryIdx].pt,
           cv::Scalar(255, 0, 0), thickness);
    }

    cv::imshow("result", output_img);
    cv::waitKey(0);

    frame_pre = std::move(frame_cur);
  }
}

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--path")) {
      option.path = argv[++i];
    } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--format")) {
      option.format = argv[++i];
    } else if (!strcmp(argv[i], "--detector")) {
      option.detector = argv[++i];
    } else if (!strcmp(argv[i], "--descriptor")) {
      option.descriptor = argv[++i];
    }
  }

  if (option.format.size() && option.path.size())
    return TestFramesInFolder(option);

  cout << "Error. Unknown arguments.\n";
  cout << "Usage: \n";
  cout << "       test\n";
  cout << "            -p, --path full_path \n";
  cout << "            -f, --format image format, e.g png, jpg\n";
  cout << "            --detector, type of detector, e.g. FAST, ORB\n";
  cout << "            --descriptor, type of descriptor, e.g. ORB, DAISY\n";
  cout << "Exampe: \n";
  cout << "./feature_tracker_app -p ~/Project/vio/data/desk_subset/ -f jpg\n";

  return 0;
}
