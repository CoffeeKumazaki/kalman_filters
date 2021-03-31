#include <stdafx.hpp>
#include <ekf.hpp>

ExtendedKF::ExtendedKF(std::shared_ptr<SystemModel> _model, const Eigen::MatrixXd& _P)
: model(_model)
, P0(_P)
, x_hat(P.rows())
{
}

ExtendedKF::~ExtendedKF() {

}

void ExtendedKF::init() {

  P = P0;
  x_hat.setZero();
  I = Eigen::MatrixXd::Identity(P.rows(), P.rows());
}

void ExtendedKF::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void ExtendedKF::predict(const Eigen::VectorXd& u) {

  x_hat = model->DynamicsModel(x_hat, u);
  Eigen::MatrixXd A_ = model->JacobDynamicsModel(x_hat, u);
  P = A_ * P * A_.transpose() + model->Q;
}

void ExtendedKF::update(const Eigen::VectorXd& y) {

  Eigen::MatrixXd C_ = model->JacobObservationModel(x_hat);
  K = (P * C_) * (C_.transpose() * P * C_ + model->R).inverse();
  x_hat += K * (y - model->ObservationModel(x_hat));
  P = (I - K * C_.transpose()) * P;
}

Eigen::VectorXd ExtendedKF::get_state() { 
  return x_hat; 
}
