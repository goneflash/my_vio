#include <opencv2/opencv.hpp>

#include <vector>

using namespace std;

class KeyframeSelector {
 public:
  KeyframeSelector() : num_matches_thres_(500) {}
  ~KeyframeSelector() {}

  bool isKeyframe(const vector<cv::DMatch> &matches) {
    if (matches.size() < num_matches_thres_) return true;

    double total_dist = 0;
    for (int i = 0; i < matches.size(); ++i) {
    }

    return false;
  }

  bool isKeyframe(const cv::Mat &img, const vector<cv::KeyPoint> &kp,
                  const vector<cv::DMatch> &matches) {
    if (matches.size() < num_matches_thres_) return true;
    return false;
  }

 private:
  int num_matches_thres_;
};
