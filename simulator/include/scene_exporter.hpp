#ifndef SCENE_EXPORTER_
#define SCENE_EXPORTER_

#include <string>

#include "scene.hpp"

namespace vio {

class SceneExporter {
 public:
  SceneExporter() {}

  bool WriteSceneToYAMLFile(const Scene &scene, const std::string file_name);
};
}  // vio

#endif  // SCENE_EXPORTER_
