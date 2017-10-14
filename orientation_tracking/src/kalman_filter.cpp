#include "kalman_filter.hpp"

void IMUKalmanFilter::updateWithMeasurement(
    const Matrix<double, 6, 1> &imu_measurement, double time_stamp) {
  double dt = time_stamp - time_stamp_;
  Matrix<double, 6, 1> predict_state;
  // Predict state
  predictState(dt, predict_state);

  // Predict error covariance
  Matrix<double, 6, 6> P_predict;
  predictErrorCovariance(P_predict);

  // Calculate Gain
  Matrix<double, 6, 6> K;
  calculateKalmanGain(P_predict, K);

  updateState(imu_measurement, predict_state, K);
  updateErrorCovariance(P_predict, K);

  time_stamp_ = time_stamp;
}

void IMUKalmanFilter::predictState(double dt,
                                   Matrix<double, 6, 1> &predict_state) {
  A_.topRightCorner(3, 3) = MatrixXd::Identity(3, 3) * dt;
  // std::cout << "Transition: \n" << A_ << std::endl;

  predict_state = A_ * imu_state_;
  //  std::cout << "Old state: \n" << imu_state_ << std::endl;
  //  std::cout << "New state: \n" << predict_state << std::endl;
}

void IMUKalmanFilter::predictErrorCovariance(Matrix<double, 6, 6> &P_predict) {
  P_predict = A_ * P_ * A_.transpose() + Q_;
}

void IMUKalmanFilter::calculateKalmanGain(const Matrix<double, 6, 6> &P_predict,
                                          Matrix<double, 6, 6> &K) {
  K = P_predict * H_.transpose() *
      (H_ * P_predict * H_.transpose() + R_).inverse();
}

void IMUKalmanFilter::updateState(const Matrix<double, 6, 1> &measurement,
                                  const Matrix<double, 6, 1> &predicted_state,
                                  const Matrix<double, 6, 6> &K) {
  imu_state_ = predicted_state + K * (measurement - H_ * predicted_state);
  //  std::cout << "Updated state: \n" << imu_state_ << std::endl;
}

void IMUKalmanFilter::updateErrorCovariance(
    const Matrix<double, 6, 6> &P_predict, const Matrix<double, 6, 6> &K) {
  P_ = (MatrixXd::Identity(6, 6) - K * H_) * P_predict;
}
