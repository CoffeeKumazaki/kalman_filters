#include <stdafx.hpp>
#include "system_model.hpp"

struct normal_random_variable
{
  normal_random_variable(Eigen::MatrixXd const& covar)
    : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
  {}

  normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
    : mean(mean)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd transform;

  Eigen::VectorXd operator()() const {
    static std::mt19937 gen{ std::random_device{}() };
    static std::normal_distribution<> dist;

    return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
  }
};

SystemModel::SystemModel(
  const Eigen::VectorXd& x0,
  const Eigen::MatrixXd& _R, // Measurement noise covariance.
  const Eigen::MatrixXd& _Q, // Process noise covariance.
  double _dt
)
: x_true(x0)
, R(_R)
, Q(_Q)
, dt(_dt)
{
}

SystemModel::~SystemModel() {
}

Eigen::VectorXd SystemModel::observation(const Eigen::VectorXd &u) {

  normal_random_variable un(Q.block(2,2,2,2));

  // std::cout << "un: \n" << un() << std::endl;

  auto ud = u + un();
  x_true = DynamicsModel(x_true, ud);

  normal_random_variable zn(R);

  auto obs = ObservationModel(x_true);

/*
  std::cout << "obs: \n"
            << obs << std::endl;
  std::cout << "zn: \n" << zn() << std::endl;
*/
  auto z = obs + zn();

  return z;
}