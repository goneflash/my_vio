#ifndef SCENE_GENERATOR_
#define SCENE_GENERATOR_

#include "scene.hpp"

namespace vio {

class SceneGenerator {
 public:
  SceneGenerator() {}

  GenerateSceneRandom(int num_landmarks, int num_views, Scene &scene);
};

} // vio
#endif // SCENE_GENERATOR_
