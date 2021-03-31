#pragma once

#include <Eigen/Dense>
#include <system_model.hpp>

class ExtendedKF {

public:
  ExtendedKF(std::shared_ptr<SystemModel> model, const Eigen::MatrixXd& P);
  ~ExtendedKF();

  void init();
  void init(const Eigen::VectorXd& x);
  void predict(const Eigen::VectorXd& u);
  void update(const Eigen::VectorXd& y);
  Eigen::VectorXd get_state();

private:
  Eigen::MatrixXd P;
  std::shared_ptr<SystemModel> model;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.
  Eigen::MatrixXd I;    // unit matrix.

  Eigen::VectorXd x_hat; // estimated state.
};