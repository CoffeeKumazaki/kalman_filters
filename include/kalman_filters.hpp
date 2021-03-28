#pragma once

#include <Eigen/Dense>

class KalmanFilter {

public:
  KalmanFilter(
    const Eigen::MatrixXd& A, // System dynamics.
    const Eigen::MatrixXd& B, // Control.
    const Eigen::MatrixXd& C, // Output.
    const Eigen::MatrixXd& P, // Estimate error covariance.
    const Eigen::MatrixXd& R, // Measurement noise covariance.
    const Eigen::MatrixXd& Q  // Process noise covariance.
    );
  ~KalmanFilter();

  void init();
  void init(const Eigen::VectorXd& x);
  void predict(const Eigen::VectorXd& u);
  void update(const Eigen::VectorXd& y);
  Eigen::VectorXd get_state();

private:
  Eigen::MatrixXd A, B, C, P, R, Q;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.
  Eigen::MatrixXd I;    // unit matrix.

  Eigen::VectorXd x_hat; // estimated state.
};