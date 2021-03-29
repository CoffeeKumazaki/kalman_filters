#include <stdafx.hpp>
#include <ekf.hpp>

ExtendedKF::ExtendedKF(
  const Eigen::MatrixXd& _P, // Estimate error covariance.
  const Eigen::MatrixXd& _R, // Measurement noise covariance.
  const Eigen::MatrixXd& _Q, // Process noise covariance.
  FuncDynamics _f,           // System Dynamics function.
  FuncObservation _h,        // Observation function.
  FuncDynamicsJacob _A,      // Jacobian of f.
  FuncObservationJacob _C    // Jacobian of h.
)
: P(_P)
, P0(_P)
, R(_R)
, Q(_Q)
, I(P.rows(), P.rows())
, x_hat(P.rows())
, f(_f)
, h(_h)
, A(_A)
, C(_C)
{
  I.setIdentity();
}

ExtendedKF::~ExtendedKF() {

}

void ExtendedKF::init() {

  P = P0;
  x_hat.setZero();
}

void ExtendedKF::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void ExtendedKF::predict(const Eigen::VectorXd& u) {

  x_hat = f(x_hat, u);
  Eigen::MatrixXd A_ = A(x_hat);
  P = A_ * P * A_.transpose() + Q;
}

void ExtendedKF::update(const Eigen::VectorXd& y) {

  Eigen::MatrixXd C_ = C(x_hat);
  K = (P * C_) * (C_.transpose() * P * C_ + R).inverse();
  x_hat += K * (y - h(x_hat));
  P = (I - K * C_) * P;
}

Eigen::VectorXd ExtendedKF::get_state() { 
  return x_hat; 
}
