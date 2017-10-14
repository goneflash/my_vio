#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

// Single axis filter. On raw pitch yaw.
// Only orientation.
class IMUKalmanFilter {
 public:
  IMUKalmanFilter() : time_stamp_(0) {
    A_ = Matrix<double, 6, 6>::Identity();
    H_ = Matrix<double, 6, 6>::Identity();
    Q_ = Matrix<double, 6, 6>::Identity();
    R_ = Matrix<double, 6, 6>::Identity();
  }
  ~IMUKalmanFilter() {}

  void getState(Matrix<double, 6, 1> &state) { state = imu_state_; }

  void updateWithMeasurement(const Matrix<double, 6, 1> &imu_measurement,
                             double time_stamp);

  void setInitialState(const Matrix<double, 6, 1> &imu_init_state) {
    imu_state_ = imu_init_state;
    std::cout << "Initial state : \n" << imu_state_ << std::endl;
  }

  void setQ(const Matrix<double, 6, 6> &Q) { Q_ = Q; }
  void setR(const Matrix<double, 6, 6> &R) { R_ = R; }

 private:
  void predictState(double dt, Matrix<double, 6, 1> &predict_state);

  void predictErrorCovariance(Matrix<double, 6, 6> &P_predict);

  void calculateKalmanGain(const Matrix<double, 6, 6> &P_predict,
                           Matrix<double, 6, 6> &K);

  void updateState(const Matrix<double, 6, 1> &measurement,
                   const Matrix<double, 6, 1> &predicted_state,
                   const Matrix<double, 6, 6> &K);

  void updateErrorCovariance(const Matrix<double, 6, 6> &P_predict,
                             const Matrix<double, 6, 6> &K);

  // [raw, pitch ,yaw, raw', pitch', yaw']
  Matrix<double, 6, 1> imu_state_;

  // State Transition Matrix
  // [ 1, 0, 0, dt,  0,  0,
  //   0, 1, 0,  0, dt,  0,
  //   0, 0, 1,  0,  0, dt,
  //   0, 0, 0,  1,  0,  0,
  //   0, 0, 0,  0,  1,  0,
  //   0, 0, 0,  0,  0,  1  ]
  Matrix<double, 6, 6> A_;
  // Observation matrix
  Matrix<double, 6, 6> H_;
  // Error Covariance
  Matrix<double, 6, 6> P_;
  // Error
  Matrix<double, 6, 6> Q_;
  Matrix<double, 6, 6> R_;

  double time_stamp_;
};
