#ifdef __linux__
#include <dirent.h>
#endif

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/viz.hpp>

bool GetImageNamesInFolder(const std::string &path, const std::string &format,
                           std::vector<std::string> &images);

void VisualizeCamerasAndPoints(const cv::Matx33d &K,
                               const std::vector<cv::Mat> &Rs,
                               const std::vector<cv::Mat> &ts,
                               const std::vector<cv::Point3f> &points);

// Allow plot multiple set of points with different color.
void VisualizeCamerasAndPoints(
    const cv::Matx33d &K, const std::vector<cv::Mat> &Rs,
    const std::vector<cv::Mat> &ts,
    const std::vector<std::vector<cv::Point3f> > &points);
