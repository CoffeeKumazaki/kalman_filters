#include <stdafx.hpp>
#include "system_simulator.hpp"

SystemSimulator::SystemSimulator(std::shared_ptr<SystemModel> _model)
: model(_model)
{

}

SystemSimulator::~SystemSimulator() {

}

void SystemSimulator::init() {
  true_results.clear();
  obs_results.clear();
}

void SystemSimulator::step(Eigen::VectorXd& u) {

  obs_results.push_back(model->observation(u));
  true_results.push_back(model->getXtrue());
}