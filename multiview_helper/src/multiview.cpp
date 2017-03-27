#include "multiview.hpp"

namespace vio {

void RtToP(const cv::Mat &R, const cv::Mat &t, cv::Mat &P) {
  P = cv::Mat(3, 4, CV_64F);
  R.copyTo(P.rowRange(0, 3).colRange(0, 3));
  t.copyTo(P.rowRange(0, 3).col(3));
}

int TriangulatePoints(const std::vector<cv::Vec2d> &kp0,
                      const std::vector<cv::Vec2d> &kp1, const cv::Mat &K,
                      const cv::Mat &R0, const cv::Mat &t0, const cv::Mat &R1,
                      const cv::Mat &t1, std::vector<cv::Point3f> &points3d,
                      std::vector<bool> &points3d_mask) {
  bool generate_test = false;
  if (generate_test) {
    cv::FileStorage file("triangulation_test_data.txt", cv::FileStorage::WRITE);

    file << "NumPoints" << (int)kp0.size();

    write(file, "kp0", kp0);
    write(file, "kp1", kp1);
    file << "K" << K << "R0" << R0 << "t0" << t0 << "R1" << R1 << "t1" << t1;
    /*
        cv::FileStorage file_read;
        file_read.open("triangulation_test_data.txt", cv::FileStorage::READ);
        int kp_num = (int)file_read["NumPoints"];
        FileNode kp0node = file_read["kp0"];

        std::vector<cv::Vec2d> kp0_new;
        std::vector<cv::Vec2d> kp1_new;
        cv::Mat K_new, R0_new, t0_new, R1_new, t1_new;

        read(file_read["kp0"], kp0_new);
        read(file_read["kp1"], kp1_new);
        file_read["K"] >> K_new;
        file_read["R0"] >> R0_new;
        file_read["t0"] >> t0_new;
        file_read["R1"] >> R1_new;
        file_read["t1"] >> t1_new;
    */
  }

  double reprojection_error_thres = 5.0;

  cv::Mat P0, P1;
  RtToP(R0, t0, P0);
  RtToP(R1, t1, P1);
  P0 = K * P0;
  P1 = K * P1;

  // TODO: Why
  cv::Mat center0 = -R0.t() * t0;
  cv::Mat center1 = -R1.t() * t1;

  int nParallal = 0;
  int nInfinite = 0;
  int nLargeError = 0;
  int nNegativeDepth = 0;
  int nGood = 0;

  points3d.resize(kp0.size());
  points3d_mask.resize(kp0.size(), true);
  for (int i = 0; i < kp0.size(); ++i) {
    TriangulateDLT(kp0[i], kp1[i], P0, P1, points3d[i]);

    cv::Mat p_global(3, 1, CV_64F);
    p_global.at<double>(0) = points3d[i].x;
    p_global.at<double>(1) = points3d[i].y;
    p_global.at<double>(2) = points3d[i].z;

    cv::Mat p3dC0 = R0 * p_global + t0;
    cv::Mat p3dC1 = R1 * p_global + t1;
    double depth0 = p3dC0.at<double>(2);
    double depth1 = p3dC1.at<double>(2);

    if (depth0 <= 0 || depth1 <= 0) {
      points3d_mask[i] = false;
      nNegativeDepth++;
      continue;
    }

    // TODO: Make sure isfinite is in std
    if (!std::isfinite(p3dC0.at<double>(0)) ||
        !std::isfinite(p3dC0.at<double>(1)) ||
        !std::isfinite(p3dC0.at<double>(2))) {
      nInfinite++;
      points3d_mask[i] = false;
      continue;
    }

    // TODO: Make sure isfinite is in std
    if (!std::isfinite(p3dC1.at<double>(0)) ||
        !std::isfinite(p3dC1.at<double>(1)) ||
        !std::isfinite(p3dC1.at<double>(2))) {
      nInfinite++;
      points3d_mask[i] = false;
      continue;
    }

    // Check parallax
    cv::Mat p0_vec = p3dC0 - center0;
    cv::Mat p1_vec = p3dC1 - center1;
    double dist0 = cv::norm(p0_vec);
    double dist1 = cv::norm(p1_vec);

    double cosParallax = p0_vec.dot(p1_vec) / (dist0 * dist1);
    if (cosParallax > 0.998) {
      nParallal++;
      points3d_mask[i] = false;
      continue;
    }

    double error0 = ComputeReprojectionError(points3d[i], kp0[i], P0);
    double error1 = ComputeReprojectionError(points3d[i], kp1[i], P1);

    if (error0 > reprojection_error_thres ||
        error1 > reprojection_error_thres) {
      nLargeError++;
      points3d_mask[i] = false;
      continue;
    }
    nGood++;
  }

  std::cout << "Found " << nGood << " / " << kp0.size()
            << " good triangulated points.\n";

  std::cout << "Found " << nInfinite << " / " << kp0.size()
            << " infinite points during triangulation.\n";
  std::cout << "Found " << nParallal << " / " << kp0.size()
            << " parallal points during triangulation.\n";
  std::cout << "Found " << nLargeError << " / " << kp0.size()
            << " large error points during triangulation.\n";
  std::cout << "Found " << nNegativeDepth << " / " << kp0.size()
            << " negative depth points during triangulation.\n";

  return nGood;
}

template <typename Point3Type>
void TriangulateDLT(const cv::Vec2d &kp1, const cv::Vec2d &kp2,
                    const cv::Mat &P1, const cv::Mat &P2, Point3Type &point3d) {
  cv::Mat A(4, 4, CV_64F);

  A.row(0) = kp1[0] * P1.row(2) - P1.row(0);
  A.row(1) = kp1[1] * P1.row(2) - P1.row(1);
  A.row(2) = kp2[0] * P2.row(2) - P2.row(0);
  A.row(3) = kp2[1] * P2.row(2) - P2.row(1);

  cv::Mat u, w, vt;
  cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

  // It's homogeneous
  cv::Mat p3d_mat = vt.row(3).t();
  point3d.x = p3d_mat.at<double>(0) / p3d_mat.at<double>(3);
  point3d.y = p3d_mat.at<double>(1) / p3d_mat.at<double>(3);
  point3d.z = p3d_mat.at<double>(2) / p3d_mat.at<double>(3);
}

double ComputeReprojectionError(const cv::Point3f &point3d, const cv::Vec2d &kp,
                                const cv::Mat &P) {
  cv::Mat point_mat(4, 1, CV_64F);
  point_mat.at<double>(0) = point3d.x;
  point_mat.at<double>(1) = point3d.y;
  point_mat.at<double>(2) = point3d.z;
  point_mat.at<double>(3) = 1.0;

  cv::Mat projected_point = P * point_mat;

  double p_x = projected_point.at<double>(0) / projected_point.at<double>(2);
  double p_y = projected_point.at<double>(1) / projected_point.at<double>(2);

  double error = (p_x - kp[0]) * (p_x - kp[0]) + (p_y - kp[1]) * (p_y - kp[1]);
  return error;
}

void Normalize(const std::vector<cv::Vec2d> &points,
               std::vector<cv::Vec2d> &normalized_points, cv::Mat &p2norm_p) {
  // Hartley, etc, p107
  // 1. The points are translated so that their centroid is at the origin.
  // 2. The points are then scaled so that the average distance from the origin
  // is equal to sqrt(2). RMS. Root Mean Square.
  // 3. Appy on two images independently

  // Libmv uses a non-isotropic scaling. p109
  double meanX = 0.0, meanY = 0.0;
  const int num_points = points.size();

  normalized_points.resize(num_points);

  for (int i = 0; i < num_points; ++i) {
    meanX += points[i][0];
    meanY += points[i][1];
  }

  meanX = meanX / num_points;
  meanY = meanY / num_points;

  double meanDevX = 0, meanDevY = 0;

  for (int i = 0; i < num_points; ++i) {
    normalized_points[i][0] = points[i][0] - meanX;
    normalized_points[i][1] = points[i][1] - meanY;

    meanDevX += normalized_points[i][0] * normalized_points[i][0];
    meanDevY += normalized_points[i][1] * normalized_points[i][1];

    //    meanDevX += abs(normalized_points[i][0]);
    //    meanDevY += abs(normalized_points[i][1]);
  }

  meanDevX = meanDevX / num_points;
  meanDevY = meanDevY / num_points;

  //  double sX = 1.0 / meanDevX, sY = 1.0 / meanDevY;
  double sX = sqrt(2.0 / meanDevX);
  double sY = sqrt(2.0 / meanDevY);

  for (int i = 0; i < num_points; ++i) {
    normalized_points[i][0] = normalized_points[i][0] * sX;
    normalized_points[i][1] = normalized_points[i][1] * sY;
  }

  p2norm_p = cv::Mat::eye(3, 3, CV_64F);
  p2norm_p.at<double>(0, 0) = sX;
  p2norm_p.at<double>(1, 1) = sY;
  p2norm_p.at<double>(0, 2) = -meanX * sX;
  p2norm_p.at<double>(1, 2) = -meanY * sY;
}

bool MakeMatrixInhomogeneous(cv::Mat &M) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      M.at<double>(i, j) = M.at<double>(i, j) / M.at<double>(2, 2);
}

bool SolveProjectionFromF(const cv::Mat &F, cv::Mat &P1, cv::Mat &P2) {
  P1 = cv::Mat::eye(3, 4, CV_64F);
  P2 = cv::Mat::zeros(3, 4, CV_64F);
  cv::Mat e2 = cv::Mat::zeros(3, 1, CV_64F);
  cv::SVD::solveZ(F.t(), e2);
  // TODO: Verify e2 is valid.
  cv::Mat P33 = P2(cv::Rect(0, 0, 3, 3));
  P33 = SkewSymmetricMatrix(e2) * F;

  e2.copyTo(P2(cv::Rect(3, 0, 1, 3)));

  std::cout << "Compute P from F...\nP1:\n" << P1 << "\nP2:\n" << P2
            << std::endl;

  return true;
}

cv::Mat SkewSymmetricMatrix(const cv::Mat &a) {
  cv::Mat sm(3, 3, CV_64F, cv::Scalar(0));
  sm.at<double>(0, 1) = -a.at<double>(2);
  sm.at<double>(0, 2) = a.at<double>(1);
  sm.at<double>(1, 0) = a.at<double>(2);
  sm.at<double>(1, 2) = -a.at<double>(0);
  sm.at<double>(2, 0) = -a.at<double>(1);
  sm.at<double>(2, 1) = a.at<double>(0);

  return sm;
}

}  // vio
