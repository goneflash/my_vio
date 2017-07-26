#include "map_initializer.hpp"

namespace vio {

MapInitializer *MapInitializer::CreateMapInitializer(
    MapInitializerOptions option) {
  switch (option.method) {
#ifdef CERES_FOUND
    case MapInitializerOptions::LIBMV:
      return CreateMapInitializerLIBMV();
#endif
    case MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL:
      return CreateMapInitializer8Point(option);
    default:
      return nullptr;
  }
}

}  // vio
