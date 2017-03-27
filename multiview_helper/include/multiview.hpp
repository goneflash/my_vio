#include <opencv2/opencv.hpp>

namespace vio {

void RtToP(const cv::Mat &R, const cv::Mat &t, cv::Mat &P);

int TriangulatePoints(const std::vector<cv::Vec2d> &kp0,
                      const std::vector<cv::Vec2d> &kp1, const cv::Mat &K,
                      const cv::Mat &R0, const cv::Mat &t0, const cv::Mat &R1,
                      const cv::Mat &t1, std::vector<cv::Point3f> &points3d,
                      std::vector<bool> &points3d_mask);

template <typename Point3Type>
void TriangulateDLT(const cv::Vec2d &kp1, const cv::Vec2d &kp2,
                    const cv::Mat &P1, const cv::Mat &P2, Point3Type &point3d);

double ComputeReprojectionError(const cv::Point3f &point3d, const cv::Vec2d &kp,
                                const cv::Mat &P);

double ComputeReprojectionError(const cv::Point3f &point3d, const cv::Vec2d &kp,
                                const cv::Mat &R, const cv::Mat &t);

void Normalize(const std::vector<cv::Vec2d> &points,
               std::vector<cv::Vec2d> &norm_points, cv::Mat &p2norm_p);

bool MakeMatrixInhomogeneous(cv::Mat &M);
bool SolveProjectionFromF(const cv::Mat &F, cv::Mat &P1, cv::Mat &P2);
cv::Mat SkewSymmetricMatrix(const cv::Mat &a);

}  // vio
