#pragma once

#include <Eigen/Dense>
using FuncDynamics = Eigen::VectorXd (*)(const Eigen::VectorXd&, const Eigen::VectorXd&);
using FuncObservation = Eigen::VectorXd (*)(const Eigen::VectorXd&);
using FuncDynamicsJacob = Eigen::MatrixXd (*)(const Eigen::VectorXd&);
using FuncObservationJacob = Eigen::MatrixXd (*)(const Eigen::VectorXd&);

class ExtendedKF {

public:
  ExtendedKF(
    const Eigen::MatrixXd& P, // Estimate error covariance.
    const Eigen::MatrixXd& R, // Measurement noise covariance.
    const Eigen::MatrixXd& Q, // Process noise covariance.
    FuncDynamics f,           // System Dynamics function.
    FuncObservation h,        // Observation function.
    FuncDynamicsJacob A,      // Jacobian of f.
    FuncObservationJacob C    // Jacobian of h.
  );
  ~ExtendedKF();

  void init();
  void init(const Eigen::VectorXd& x);
  void predict(const Eigen::VectorXd& u);
  void update(const Eigen::VectorXd& y);
  Eigen::VectorXd get_state();

private:
  FuncDynamics f;
  FuncObservation h;
  FuncDynamicsJacob A;
  FuncObservationJacob C;
  Eigen::MatrixXd P, R, Q;
  Eigen::MatrixXd P0;   // initial P.
  Eigen::MatrixXd K;    // kalman gain.
  Eigen::MatrixXd I;    // unit matrix.

  Eigen::VectorXd x_hat; // estimated state.

};