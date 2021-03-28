#include <stdafx.hpp>
#include <kalman_filters.hpp>

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& _A, // System dynamics.
    const Eigen::MatrixXd& _B, // Control.
    const Eigen::MatrixXd& _C, // Output.
    const Eigen::MatrixXd& _P, // Estimate error covariance.
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q  // Process noise covariance.
)
: A(_A)
, B(_B)
, C(_C)
, P(_P)
, P0(_P)
, R(_R)
, Q(_Q)
, I(A.rows(), A.rows())
, x_hat(A.rows())
{
  I.setIdentity();
}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::init() {

  P = P0;
  x_hat.setZero();
}

void KalmanFilter::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void KalmanFilter::predict(const Eigen::VectorXd& u) {

  x_hat = A * x_hat + B * u;
  P = A*P*A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat += K * (y - C*x_hat);
	P = (I - K*C)*P;
}

Eigen::VectorXd KalmanFilter::get_state() { 
  return x_hat; 
}
