#include <stdafx.hpp>
#include "system_simulator.hpp"

SystemSimulator::SystemSimulator() {

}

SystemSimulator::~SystemSimulator() {

}

void SystemSimulator::init_model(
  const Eigen::VectorXd& x0,
  const Eigen::MatrixXd& _R, // Measurement noise covariance.
  const Eigen::MatrixXd& _Q, // Process noise covariance.
  double dt
)
{
  if (model) {
    model.reset();
  }
  model = std::make_shared<SystemModelA>(x0, _R, _Q, dt);
  true_results.clear();
  obs_results.clear();
}

void SystemSimulator::step(Eigen::VectorXd& u) {

  obs_results.push_back(model->observation(u));
  true_results.push_back(model->getXtrue());
}