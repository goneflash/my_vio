#include "map_initializer.hpp"

namespace vio {

MapInitializer *MapInitializer::CreateMapInitializer(
    MapInitializerOptions option) {
  switch (option.method) {
    case MapInitializerOptions::LIBMV:
      return CreateMapInitializerLIBMV();
    case MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL:
      return CreateMapInitializer8Point(option);
    default:
      return nullptr;
  }
}

}  // vio
