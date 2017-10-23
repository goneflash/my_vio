#include "map_initializer.hpp"

namespace vio {

std::unique_ptr<MapInitializer> MapInitializer::CreateMapInitializer(
    MapInitializerOptions option) {
  switch (option.method) {
#ifdef SFM_FOUND
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
