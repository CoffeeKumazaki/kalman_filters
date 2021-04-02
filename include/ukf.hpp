#pragma once

#include <Eigen/Dense>
#include <system_model.hpp>

class UnscentedKF {
public:
  UnscentedKF(std::shared_ptr<SystemModel> model, const Eigen::MatrixXd& P, double scale = 0.0);
  ~UnscentedKF();

  void init();
  void init(const Eigen::VectorXd& x);
  void predict(const Eigen::VectorXd &u);
  void update(const Eigen::VectorXd& y);
  Eigen::VectorXd get_state();
  Eigen::MatrixXd get_cov();

private:
  void ut(const Eigen::VectorXd& _x, const Eigen::MatrixXd& _P, std::vector<Eigen::VectorXd>& sigmaPt);

private:
  Eigen::MatrixXd P;
  std::shared_ptr<SystemModel> model;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.
  Eigen::MatrixXd I;    // unit matrix.

  Eigen::VectorXd x_hat; // estimated state.

  std::vector<double> wc, ws; // weight.
  double lambda;
};