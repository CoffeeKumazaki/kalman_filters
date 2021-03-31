#pragma once

#include <Eigen/Dense>

class SystemModel {

public:
  SystemModel(
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q, // Process noise covariance.
    double _dt = 1.0/30.0    
    );
  ~SystemModel();

  virtual Eigen::VectorXd DynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u) = 0;
  virtual Eigen::VectorXd ObservationModel(const Eigen::VectorXd &x) = 0;
  virtual Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u) = 0;
  virtual Eigen::MatrixXd JacobObservationModel(const Eigen::VectorXd &x) = 0;

  Eigen::VectorXd observation(const Eigen::VectorXd &u);

  Eigen::VectorXd getXtrue() { return x_true; }

  Eigen::MatrixXd R, Q;
protected:
  Eigen::VectorXd x_true;
  double dt;
};