#include "graph_optimizer.hpp"

namespace vio {

std::unique_ptr<GraphOptimizer> GraphOptimizer::CreateGraphOptimizer(
    GraphOptimizerMethod method) {
  switch (method) {
    case CERES:
      return CreateGraphOptimizerCeres();
    default:
      return nullptr;
  }
}

}  // vio
