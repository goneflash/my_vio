#ifndef IMU_INTEGRATOR_IMU_INTEGRATOR_HPP_
#define IMU_INTEGRATOR_IMU_INTEGRATOR_HPP_

#include <Eigen/Dense>

namespace vio {

class ImuIntegrator {
 public:
  ImuIntegrator() {}

  bool ZerothOrderIntegration(const Eigen::Quaterniond &start_q,
                              const Eigen::Vector3d &omega,
                              double delta_t,
                              Eigen::Quaterniond &end_q);
 private:  
};

} // vio

#endif

