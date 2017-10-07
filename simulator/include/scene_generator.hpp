#ifndef SCENE_GENERATOR_
#define SCENE_GENERATOR_

#include "scene.hpp"

namespace vio {

class SceneGenerator {
 public:
  SceneGenerator() {}

  void GenerateSceneRandom(int num_landmarks, int num_views, Scene &scene);

  // TODO
  // AddTranslationTrajectory
  // AddRotationTrajectory

  // TODO
  // AddOutliers
  // RandomlyMovePointsAround
  // RemoveSomeLandmarksInEachView

 private:
  void GenerateLandmarks(int num_landmarks, Scene &scene);
  // Generate smooth trajectory of views.
  void GenerateTrajectory(int num_views, Scene &scene);
  // Generate separte views in the space.
  void GenerateViews(int num_views, Scene &scene);
};

}  // vio
#endif  // SCENE_GENERATOR_
