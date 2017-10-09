#include "scene_generator.hpp"

#include <opencv2/core.hpp>

namespace vio {

void SceneGenerator::GenerateSceneRandom(int num_landmarks, int num_views,
                                         Scene &scene) {
  GenerateLandmarks(num_landmarks, scene);
  GenerateViews(num_views, scene);

  // TODO: Add P
  // https://github.com/opencv/opencv_contrib/blob/246ea8f3bdf174a2aad6216c2601e3a93bf75c29/modules/sfm/test/scene.cpp
}

void SceneGenerator::GenerateLandmarks(int num_landmarks, Scene &scene) {
  scene.landmarks.resize(num_landmarks);

  cv::RNG rng;
  cv::Mat_<double> points3d;
  // Generate a bunch of random 3d points in a 0, 1 cube
  points3d.create(num_landmarks, 3);
  rng.fill(points3d, cv::RNG::UNIFORM, 0, 1);
  // Copy to scene.
  for (int i = 0; i < num_landmarks; ++i) {
    scene.landmarks[i].position[0] = points3d.at<double>(i, 0);
    scene.landmarks[i].position[1] = points3d.at<double>(i, 1);
    scene.landmarks[i].position[2] = points3d.at<double>(i, 2);
  }
}

void SceneGenerator::GenerateViews(int num_views, Scene &scene) {
  scene.trajectory.resize(num_views);

  // Set first camera.
  scene.trajectory[0].R = cv::Mat::eye(3, 3, CV_64F);
  scene.trajectory[0].t = cv::Vec3d(0, 0, 0);

  cv::RNG rng;
  for (int i = 1; i < num_views; ++i) {
    // ----------------- Add random rotation
    // Get random rotation axis.
    cv::Vec3d vec;
    rng.fill(vec, cv::RNG::UNIFORM, 0, 1);
    // Give a random angle to the rotation vector.
    vec = vec / cv::norm(vec) * rng.uniform(0.0f, float(CV_PI));
    cv::Rodrigues(vec, scene.trajectory[i].R);
    // ----------------- Add random translation
    scene.trajectory[i].t =
        cv::Vec3d(rng.uniform(-0.5f, 0.5f), rng.uniform(-0.5f, 0.5f),
                  rng.uniform(1.0f, 2.0f));
    std::cout << "Random view generated.\nR:\n" << scene.trajectory[i].R
              << "\nt:\n" << scene.trajectory[i].t << std::endl;
    // -------------- Make sure all points is in front of the camera
    double min_dist;
    for (int ld = 0; ld < scene.landmarks.size(); ++ld) {
      cv::Mat point = cv::Mat(3, 1, CV_64F);
      point.at<double>(0) = scene.landmarks[ld].position[0];
      point.at<double>(1) = scene.landmarks[ld].position[1];
      point.at<double>(2) = scene.landmarks[ld].position[2];
      cv::Mat transformed_point =
          scene.trajectory[i].R * point + cv::Mat(scene.trajectory[i].t);
    
      if (ld == 0)
        min_dist = transformed_point.at<double>(2);
      else
        min_dist = min_dist > transformed_point.at<double>(2)
                       ? transformed_point.at<double>(2)
                       : min_dist;
    }
    if (min_dist < 0)
      scene.trajectory[i].t[2] = scene.trajectory[i].t[2] - min_dist + 1.0;
  }
}

void SceneGenerator::GenerateTrajectory(int num_views, Scene &scene) {}

}  // vio
