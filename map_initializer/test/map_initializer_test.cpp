#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "config.hpp"
#include "map_initializer.hpp"

#ifdef OPENCV_VIZ_FOUND
#include "scene_visualizer.hpp"
#endif

class MapInitializerTest : public ::testing::Test {
 protected:
  void CreateInitializer() {
    map_initializer_ = vio::MapInitializer::CreateMapInitializer(options_);
  }

  void GetTwoFrameFeatureTracks() {
    cv::Mat_<double> x1, x2;
    int num_pts;
    std::ifstream myfile(root_path +
                         "/map_initializer/test/test_data/feature_tracks.txt");
    ASSERT_TRUE(myfile.is_open());

    feature_vectors_.resize(2);
    std::string line;

    // TODO: Put this to util for other use cases.
    // Read number of points
    getline(myfile, line);
    num_pts = (int)atof(line.c_str());

    // feature_vectors_[0].resize(num_pts);
    // feature_vectors_[1].resize(num_pts);

    feature_vectors_[0].clear();
    feature_vectors_[1].clear();

    x1 = cv::Mat_<double>(2, num_pts);
    x2 = cv::Mat_<double>(2, num_pts);

    // Read the point coordinates
    for (int i = 0; i < num_pts; ++i) {
      getline(myfile, line);
      std::stringstream s(line);
      std::string cord;
      s >> cord;
      x1(0, i) = atof(cord.c_str());
      s >> cord;
      x1(1, i) = atof(cord.c_str());
      s >> cord;
      x2(0, i) = atof(cord.c_str());
      s >> cord;
      x2(1, i) = atof(cord.c_str());

      // Skip missing features.
      if (x1(0, i) < 0 || x1(1, i) < 0 || x2(0, i) < 0 || x2(1, i) < 0)
        continue;

      feature_vectors_[0].push_back(cv::Vec2d(x1(0, i), x1(1, i)));
      feature_vectors_[1].push_back(cv::Vec2d(x2(0, i), x2(1, i)));

      /*
      feature_vectors_[0][i][0] = x1(0, i);
      feature_vectors_[0][i][1] = x1(1, i);
      feature_vectors_[1][i][0] = x2(0, i);
      feature_vectors_[1][i][1] = x2(1, i);
      */
    }
    myfile.close();
    K_ = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  }

  // Sigma is in number of pixel.
  void AddNoiseToFeatureTracks(double sigma) {
    cv::RNG rng;
    for (auto &features : feature_vectors_) {
      for (auto &feature : features) {
        feature[0] += rng.gaussian(sigma);
        feature[1] += rng.gaussian(sigma);
      }
    }
  }
  /*
    void GetFeatureVectorFromTracks() {
      std::ifstream
    myfile("../map_initializer/test/test_data/backyard_tracks.txt");
      ASSERT_TRUE(myfile.is_open());
      double x, y;
      std::string line_str;
      int num_frames = 0, n_tracks = 0;
      std::vector<std::vector<cv::Vec2d> > tracks;

      for ( ; getline(myfile,line_str); ++n_tracks) {
        std::istringstream line(line_str);
        std::vector<cv::Vec2d> track;
        for ( num_frames = 0; line >> x >> y; ++num_frames) {
          if ( x > 0 && y > 0)
            track.push_back(cv::Vec2d(x,y));
          else
            track.push_back(cv::Vec2d(-1));
        }
        tracks.push_back(track);
      }
      for (int i = 0; i < num_frames; ++i) {
        cv::Mat frame(cv::Size(2, n_tracks));
        for (int j = 0; j < n_tracks; ++j)
        {
          frame(0,j) = tracks[j][i][0];
          frame(1,j) = tracks[j][i][1];
        }
        points2d.push_back(frame);
      }
      myfile.close();
      K_ = cv::Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }
  */
  void RunInitializer() {
    std::vector<cv::Point3f> points3d;
    std::vector<bool> points3d_mask;
    std::vector<cv::Mat> Rs_est, ts_est;

    ASSERT_TRUE(map_initializer_->Initialize(feature_vectors_, cv::Mat(K_),
                                             points3d, points3d_mask, Rs_est,
                                             ts_est));
    int valid_point_count = 0;
    for (const auto mask : points3d_mask)
      if (mask) valid_point_count++;

    // Must have 50% triangulated points.
    ASSERT_GE(valid_point_count, feature_vectors_[0].size() * 0.5);

    std::cout << "Triangulated " << valid_point_count << " / "
              << feature_vectors_[0].size() << " points.\n";

    /*
#ifdef OPENCV_VIZ_FOUND
    vio::SceneVisualizer my_viz("initializer");
    my_viz.SetLandmarks(points3d);
    my_viz.Render();
#endif
*/
  }

  vio::MapInitializer *map_initializer_;
  vio::MapInitializerOptions options_;

  std::vector<std::vector<cv::Vec2d> > feature_vectors_;

  cv::Matx33d K_;
};

#ifdef SFM_FOUND
TEST_F(MapInitializerTest, TestLibmv_TwoFrame) {
  options_.method = vio::MapInitializerOptions::LIBMV;
  CreateInitializer();
  GetTwoFrameFeatureTracks();

  RunInitializer();
}
#endif

TEST_F(MapInitializerTest, Test8Point_TwoFrame) {
  options_.method = vio::MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL;
  CreateInitializer();
  GetTwoFrameFeatureTracks();

  RunInitializer();
}

TEST_F(MapInitializerTest, Test8Point_TwoFrame_Noise) {
  options_.method = vio::MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL;
  CreateInitializer();
  GetTwoFrameFeatureTracks();
  AddNoiseToFeatureTracks(3.0);

  RunInitializer();
}

TEST_F(MapInitializerTest, Test8Point_TwoFrame_OpenCVRansacF) {
  options_.method = vio::MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL;
  options_.use_f_ransac = true;
  CreateInitializer();
  GetTwoFrameFeatureTracks();

  RunInitializer();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
